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
    public double kP = 1 / 2500.0;
    public double kI = 0.0002;
    public double kD = 0.1 / 14000;

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

    public static final double HUB_HOOD = 175;
    public static final double HUB_RPM = 4300;

    public static final double CORNER_HOOD = 194;
    public static final double CORNER_RPM = 5300;

    public static final double TRENCH_HOOD = 191;
    public static final double TRENCH_RPM = 4880;

    public static final double TOWER_HOOD = 187;
    public static final double TOWER_RPM = 4300;
    public static final double TOWER_TURRET = 273;


    public ShooterTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.joystick = joystick;
        shooterSubsystem = robot.getShooterSubsystem();


        shooterPIDController.setSetpoint(targetRPM);
        shooterPIDController.setTolerance(300); // Dont know if this is needed but a fine safety net ig
       // shooterPIDController.setIntegratorRange(0,0.6);
        addRequirements(shooterSubsystem);

    }


    @Override
    public void initialize() {
        //shooterPIDController.reset(); isnt needed right now since kI and kD are zero
        turretSetpoint = shooterSubsystem.getTurretAngleinDegrees().in(Degrees);
        hoodSetpoint = shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees);
        shooterSetpoint = 4300;
        hoodPIDController.setSetpoint(hoodSetpoint);
        turretPIDController.setSetpoint(turretSetpoint);

    }

    @Override
    public void execute() {
       /*
        //hub

        if (joystick.getRawButtonPressed(10)) {
            shooterSetpoint = 4300;
            hoodSetpoint = 175;
            isFlywheelOn = true;
        }
        //corner
        if (joystick.getRawButtonPressed(12)) {
            shooterSetpoint = 5300;
            hoodSetpoint = 194;
            isFlywheelOn = true;
        }
        //trench
        if (joystick.getRawButtonPressed(13)) {
            shooterSetpoint = 4880;
            hoodSetpoint = 191;
            isFlywheelOn = true;
        }
        //tower
        if (joystick.getRawButtonPressed(14)) {
            shooterSetpoint = 4300;
            hoodSetpoint = 187;
            turretSetpoint = 273;
            isFlywheelOn = true;
        }



        */


        // Flywheel Toggle
        if (joystick.getRawButtonPressed(2)) {
            isFlywheelOn = !isFlywheelOn;

        }

        if (joystick.getRawButton(3)) {
            shooterSetpoint -= 10;
        }

        if (joystick.getRawButton(4)) {
            shooterSetpoint += 10;
        }

        Logger.recordOutput("FlywheelSetpoint", shooterSetpoint);
        shooterPIDController.setSetpoint(shooterSetpoint);


        if (isFlywheelOn) {

            AngularVelocity currentRPM = shooterSubsystem.getFlywheelSpeed();

            double output = shooterPIDController.calculate(currentRPM.in(RPM), shooterSetpoint);

            Logger.recordOutput("FlywheelOutput", output);
            Logger.recordOutput("FlywheelI", shooterPIDController.getAccumulatedError() * kI);

            if (output > 0.55) output = 0.55;
            if (output < 0) output = 0;

            shooterSubsystem.setFlywheelPower(0.45+ output);
            shooterSubsystem.setTransitionPower(0.7);

        } else {
            shooterSubsystem.setTransitionPower(0);
            shooterSubsystem.setFlywheelPower(0);
        }
        shooterSubsystem.getSensors().atShooterSpeed = shooterPIDController.atSetpoint();



        // Shoot Button
        if (joystick.getRawAxis(3) > 0.3) {


            if (true
                            && isFlywheelOn
//                    && shooterPIDController.atSetpoint()
            ) {
                shooterSubsystem.setSpindexerPower(1);
            } else {
                shooterSubsystem.setSpindexerPower(0);
            }
        } else if(joystick.getRawAxis(2)> 0.3){
            shooterSubsystem.setSpindexerPower(-1);
            if(!isFlywheelOn){
                shooterSubsystem.setTransitionPower(-0.7);
            }
        } else {
            shooterSubsystem.setSpindexerPower(0);
        }

        if (joystick.getRawAxis(0) > deadZone) { // && !shooterSubsystem.getSensors().reachedMaxHood
            turretSetpoint -= 0.5;

        }

        if (joystick.getRawAxis(0) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            turretSetpoint += 0.5;

        }
        Logger.recordOutput("preClampTurretSetpoint",turretSetpoint);
        turretSetpoint = MathUtil.clamp(
                turretSetpoint,
                shooterSubsystem.getSensors().turretMinAngle,
                shooterSubsystem.getSensors().turretMaxAngle);


        turretPIDController.setSetpoint(turretSetpoint);
        Logger.recordOutput("turretSetpoint", turretPIDController.getSetpoint());
        Logger.recordOutput("turretRotation", shooterSubsystem.getTurretAngleinDegrees().in(Degrees));
        Logger.recordOutput("shootetAtSetPoint", shooterPIDController.atSetpoint());


        double output = turretPIDController.calculate(
                shooterSubsystem.getTurretAngleinDegrees().in(Degrees),
                turretSetpoint);

        if(output >0.4){
            output=0.4;
        }




        Logger.recordOutput("turretOutput", output);
        shooterSubsystem.setTurretPower(output);


        if (joystick.getRawAxis(5) > deadZone) { // && !shooterSubsystem.getSensors().reachedMa5xHood
            hoodSetpoint -= 2/10.0;
        }

        if (joystick.getRawAxis(5) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            hoodSetpoint += 2/10.0;
        }

        hoodSetpoint = MathUtil.clamp(
                hoodSetpoint,
                shooterSubsystem.getSensors().hoodMinAngle,
                shooterSubsystem.getSensors().hoodMaxAngle);

        hoodPIDController.setSetpoint(hoodSetpoint);

        Logger.recordOutput("hoodSetpoint", hoodPIDController.getSetpoint());
        Logger.recordOutput("hoodRotation", shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));

        double outputHood = hoodPIDController.calculate(shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));


        if(outputHood > 0.4){
            outputHood =0.4;
        }

        Logger.recordOutput("hoodOutput", outputHood);
        shooterSubsystem.setHoodPower(outputHood);




    }



    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelPower(0);
        shooterSubsystem.setTurretPower(0);
        shooterSubsystem.setHoodPower(0);
        shooterSubsystem.stopFeeding();

    }


    @Override
    public boolean isFinished() {
        return false;
    }

}

