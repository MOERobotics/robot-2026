package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class ShooterTeleopCommand extends Command {

    public final ShooterSubsystem shooterSubsystem;
    public final Joystick joystick;
    double flywheelPower;
    AngularVelocity targetFlywheelPower;
    boolean isFlywheelOn = false;
    public double kP = 2.0 / 1500.0;
    public double kI = 0.00015;
    public double kD = 0.6 / 14000;
    public double IZone = 1000;

    public double hoodKP = 0.056;
    public double hoodKI = 0;
    public double hoodKD = 0;

    public double deadZone = 0.4;

    public double turretKP = 0.0055;
    public double turretKI = 0;
    public double turretKD = 0;
    private static final double targetRPM = 5000;
    PIDController shooterPIDController = new PIDController(kP, kI, kD);
    PIDController hoodPIDController = new PIDController(hoodKP, hoodKI, hoodKD);
    PIDController turretPIDController = new PIDController(turretKP, turretKI, turretKD);

    public double turretSetpoint, hoodSetpoint, shooterSetpoint;

    // setpoints were changed many times (not sure how accurate)
    public static final double HUB_HOOD = 179;
    public static final double HUB_RPM = 3800;
    public static final double HUB_TURRET = 273;


    public static final double CORNER_HOOD = 193.11;
    public static final double CORNER_RPM = 4990;
    public static final double CORNER_TURRET = 273;


    public static final double TRENCH_HOOD = 193.11;
    public static final double TRENCH_RPM = 3910;
    public static final double TRENCH_TURRET = 74;


    public static final double TOWER_HOOD = 194.14;
    public static final double TOWER_RPM = 3810;
    public static final double TOWER_TURRET = 327.18;


    int noPOV = -1;

    public ShooterTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.joystick = joystick;
        shooterSubsystem = robot.getShooterSubsystem();


        shooterPIDController.setSetpoint(targetRPM);
        shooterPIDController.setTolerance(100); // Dont know if this is needed but a fine safety net ig
        // shooterPIDController.setIntegratorRange(0,0.6);
        addRequirements(shooterSubsystem);

    }


    @Override
    public void initialize() {
        //shooterPIDController.reset(); isnt needed right now since kI and kD are zero
        turretSetpoint = shooterSubsystem.getSensors().turretRelativeAngleDegrees;
        hoodSetpoint = shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees);
        shooterSetpoint = 4300;
        hoodPIDController.setSetpoint(hoodSetpoint);
        turretPIDController.setSetpoint(turretSetpoint);
        shooterPIDController.reset();
        shooterPIDController.setIZone(IZone);
        shooterPIDController.setIntegratorRange(-.05, .05);
        Logger.recordOutput("turretSetpoint", turretSetpoint);
    }

    @Override
    public void execute() {

        int currentPOV = joystick.getPOV();
        // removed turret movement from presets as of Roshik's request
        if (currentPOV != -1) {
            switch (currentPOV) {
                case 0:
                    shooterSetpoint = HUB_RPM;
                    hoodSetpoint = HUB_HOOD;
                    // turretSetpoint = HUB_TURRET;
                    isFlywheelOn = true;
                    break;
                case 90:
                    shooterSetpoint = TRENCH_RPM;
                    hoodSetpoint = TRENCH_HOOD;
                    // turretSetpoint = TRENCH_TURRET;
                    isFlywheelOn = true;
                    break;

                case 180:
                    shooterSetpoint = TOWER_RPM;
                    hoodSetpoint = TOWER_HOOD;
                    // turretSetpoint = TOWER_TURRET;
                    isFlywheelOn = true;
                    break;

                case 270:
                    shooterSetpoint = CORNER_RPM;
                    hoodSetpoint = CORNER_HOOD;
                    // turretSetpoint = CORNER_TURRET;
                    isFlywheelOn = true;
                    break;

            }
        }




        // Flywheel Toggle
        if (joystick.getRawButtonPressed(2)) {
            isFlywheelOn = !isFlywheelOn;
            shooterPIDController.reset();

        }

        if (joystick.getRawButton(3)) {
            shooterSetpoint -= 2;
        }

        if (joystick.getRawButton(4)) {
            shooterSetpoint += 2;
        }

        Logger.recordOutput("FlywheelSetpoint", shooterSetpoint);
        Logger.recordOutput("FlywheelSpeed", shooterSubsystem.getFlywheelSpeed().in(RPM));
        shooterPIDController.setSetpoint(shooterSetpoint);


        if (isFlywheelOn) {

            AngularVelocity currentRPM = shooterSubsystem.getFlywheelSpeed();

            double output = shooterPIDController.calculate(currentRPM.in(RPM), shooterSetpoint);

            Logger.recordOutput("FlywheelOutput", output);
            Logger.recordOutput("FlywheelI", shooterPIDController.getAccumulatedError() * kI);

            double feedforward = shooterSubsystem.feedForwardCalc(shooterSetpoint);
            double outputMax = 1 - feedforward;
            if (output > outputMax) output = outputMax;
            if (output < 0) output = 0;


            shooterSubsystem.setFlywheelPower(feedforward + output);
            Logger.recordOutput("Flywheelff", feedforward);

            shooterSubsystem.setFlywheelPower(feedforward + output);
            shooterSubsystem.setTransitionPower(0.7);

        } else {
            shooterSubsystem.setTransitionPower(0);
            shooterSubsystem.setFlywheelPower(0);
        }
        shooterSubsystem.getSensors().atShooterSpeed = shooterPIDController.atSetpoint();


        // Shoot Button
        if (joystick.getRawAxis(3) > 0.3) {


            if (true
                // && isFlywheelOn
//                    && shooterPIDController.atSetpoint()
            ) {
                shooterSubsystem.setSpindexerPower(0.75);
            } else {
                shooterSubsystem.setSpindexerPower(0);
            }
        } else if (joystick.getRawAxis(2) > 0.3) {
            shooterSubsystem.setSpindexerPower(-1);
            shooterSubsystem.setTransitionPower(-0.6);


        } else {
            shooterSubsystem.setSpindexerPower(0);
        }

        if (joystick.getRawAxis(0) > deadZone) { // && !shooterSubsystem.getSensors().reachedMaxHood
            turretSetpoint -= 5;

        }

        if (joystick.getRawAxis(0) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            turretSetpoint += 5;

        }
        Logger.recordOutput("preClampTurretSetpoint", turretSetpoint);
        turretSetpoint = MathUtil.clamp(
                turretSetpoint,
                shooterSubsystem.getSensors().turretMinAngle,
                shooterSubsystem.getSensors().turretMaxAngle);


        turretPIDController.setSetpoint(turretSetpoint);
        Logger.recordOutput("turretSetpoint", turretPIDController.getSetpoint());
        Logger.recordOutput("turretRotation", shooterSubsystem.getSensors().turretRelativeAngle.in(Degrees));
        Logger.recordOutput("shootetAtSetPoint", shooterPIDController.atSetpoint());


        double output = turretPIDController.calculate(
                shooterSubsystem.getSensors().turretRelativeAngle.in(Degrees),
                turretSetpoint);

        if (output > 0.4) {
            output = 0.4;
        }


        Logger.recordOutput("turretOutput", output);
        shooterSubsystem.setTurretPower(output);


        if (joystick.getRawAxis(5) > deadZone) { // && !shooterSubsystem.getSensors().reachedMa5xHood
            hoodSetpoint -= 2 / 8.0;
        }

        if (joystick.getRawAxis(5) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            hoodSetpoint += 2 / 8.0;
        }
        hoodSetpoint = MathUtil.clamp(
                hoodSetpoint,
                shooterSubsystem.getSensors().hoodMinAngle,
                shooterSubsystem.getSensors().hoodMaxAngle);

        hoodPIDController.setSetpoint(hoodSetpoint);

        Logger.recordOutput("hoodSetpoint", hoodPIDController.getSetpoint());
        Logger.recordOutput("hoodRotation", shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));

        double outputHood = hoodPIDController.calculate(shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));
        if (outputHood > 0.4) {
            outputHood = 0.4;
        }

        Logger.recordOutput("hoodOutput", outputHood);
        shooterSubsystem.setHoodPower(outputHood);
        shooterSubsystem.setRampPower(0.7);


    }


    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelPower(0);
        shooterSubsystem.setRampPower(0.7);
        shooterSubsystem.setTurretPower(0);
        shooterSubsystem.setHoodPower(0);
        shooterSubsystem.stopFeeding();

    }


    @Override
    public boolean isFinished() {
        return false;
    }

}

