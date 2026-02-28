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
    public double kP = 1 / 2000.0;
    public double kI = 0.0001;
    public double kD = 0.1 / 17500;

    public double hoodKP = 0.05;
    public double hoodKI = 0;
    public double hoodKD = 0;

    public double deadZone = 0.4;

    public double turretKP = 0.5;
    public double turretKI = 0;
    public double turretKD = 0;
    private static final double targetRPM = 3000;
    PIDController shooterPIDController = new PIDController(kP, kI, kD);
    PIDController hoodPIDController = new PIDController(hoodKP, hoodKI, hoodKD);
    PIDController turretPIDController = new PIDController(turretKP, turretKI, turretKD);

    public double turretSetpoint, hoodSetpoint;
    public double turretTurnMagnitude = 5;
    public double hoodTurnMagnitude = 5;
    public boolean leftTurretShift, rightTurretShift;
    public boolean leftHoodShift, rightHoodShift;

    public ShooterTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.joystick = joystick;
        shooterSubsystem = robot.getShooterSubsystem();
        this.flywheelPower = flywheelPower;

        rightTurretShift = true;
        leftTurretShift = true;

        shooterPIDController.setSetpoint(targetRPM);
        shooterPIDController.setTolerance(100); // Dont know if this is needed but a fine safety net ig

        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize() {
        //shooterPIDController.reset(); isnt needed right now since kI and kD are zero
        turretSetpoint = shooterSubsystem.getTurretAngleinDegrees().in(Degrees);
        hoodSetpoint = shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees);
        hoodPIDController.setSetpoint(shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));
        turretPIDController.setSetpoint(turretSetpoint);

    }

    @Override
    public void execute() {
        // Flywheel Toggle
        if (joystick.getRawButtonPressed(2)) {
            isFlywheelOn = !isFlywheelOn;

        }
        if (isFlywheelOn) {
            AngularVelocity currentRPM = shooterSubsystem.getFlywheelSpeed();
            double output = shooterPIDController.calculate(currentRPM.in(RPM));
            Logger.recordOutput("FlywheelOutput", output);
            Logger.recordOutput("FlywheelI", shooterPIDController.getAccumulatedError());
            if (output > 0.5) output = 0.5;
            if (output < 0) output = 0;
            shooterSubsystem.setFlywheelPower(0.5 + output);
        } else {
            shooterSubsystem.setFlywheelPower(0);
        }

        // Shoot Button
        if (joystick.getRawAxis(3) > 0.3) {
            if (isFlywheelOn && shooterPIDController.atSetpoint()) {

                shooterSubsystem.setSpindexerPower(0.2);
                shooterSubsystem.setTransitionPower(0.2);
            } else {
                shooterSubsystem.stopFeeding();
            }
        } else {
            shooterSubsystem.stopFeeding();
        }

        if (joystick.getRawAxis(0) > deadZone) { // && !shooterSubsystem.getSensors().reachedMaxHood
            turretSetpoint -= 0.1;
            /*
            if (rightTurretShift) {
                rightTurretShift = false;
                turretSetpoint -= 5;
            }

             */
        }

        if (joystick.getRawAxis(0) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            turretSetpoint += 0.1;
            /*
            if (leftTurretShift) {
                leftTurretShift = false;
                turretSetpoint += 5;
            }

             */
        }
        Logger.recordOutput("preClampTurretSetpoint",turretSetpoint);
        turretSetpoint = MathUtil.clamp(
                turretSetpoint,
                shooterSubsystem.getSensors().turretMinAngle,
                shooterSubsystem.getSensors().turretMaxAngle
        );
        turretPIDController.setSetpoint(turretSetpoint);
        Logger.recordOutput("turretSetpoint", turretPIDController.getSetpoint());
        Logger.recordOutput("turretRotation", shooterSubsystem.getTurretAngleinDegrees().in(Degrees));

        double output = turretPIDController.calculate(
                shooterSubsystem.getTurretAngleinDegrees().in(Degrees),
                turretSetpoint);
        Logger.recordOutput("turretOutput", output);
        shooterSubsystem.setTurretPower(output);

//        if (joystick.getRawAxis(0) >= -deadZone && joystick.getRawAxis(0) <= deadZone) {
//         //   turretSetpoint = shooterSubsystem.getTurretAngleinDegrees().in(Degrees);
//            shooterSubsystem.setTurretPower(0);
//            rightTurretShift = true;
//            leftTurretShift = true;
//        }


        if (joystick.getRawAxis(5) > deadZone) { // && !shooterSubsystem.getSensors().reachedMaxHood
            hoodSetpoint -= 1/100.0;
            /*
            if (rightHoodShift) {
                rightHoodShift = false;
                hoodSetpoint -= 5;
            }

             */
        }

        if (joystick.getRawAxis(5) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            hoodSetpoint += 1/100.0;
            /*
            if (leftHoodShift) {
                leftHoodShift = false;
                hoodSetpoint += 5;
            }

             */
        }
        //Logger.recordOutput("preClampTurretSetpoint",turretSetpoint);
        hoodSetpoint = MathUtil.clamp(
                hoodSetpoint,
                shooterSubsystem.getSensors().hoodMinAngle,
                shooterSubsystem.getSensors().hoodMaxAngle
        );
        hoodPIDController.setSetpoint(hoodSetpoint);
        Logger.recordOutput("hoodSetpoint", hoodPIDController.getSetpoint());
        Logger.recordOutput("hoodRotation", shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));

        double outputHood = hoodPIDController.calculate(
                shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees),
                hoodSetpoint);
        Logger.recordOutput("hoodOutput", outputHood);
        shooterSubsystem.setHoodPower(outputHood);
/*
        if (joystick.getRawAxis(5) >= -deadZone && joystick.getRawAxis(5) <= deadZone) {
            hoodSetpoint = shooterSubsystem.getHoodAngleinDegrees().in(Degrees);
            shooterSubsystem.setHoodPower(0);
            rightHoodShift = true;
            leftHoodShift = true;
        }

 */

    }

    // will need to make what we are doing for turret and hood but as its own function.
    public void turnIncremental(double deadZone,
                                  boolean rightShift,
                                  boolean leftShift,
                                  PIDController pidController,
                                  double setpoint,
                                  double magnitude){

        if (joystick.getRawAxis(0) > deadZone) { // && !shooterSubsystem.getSensors().reachedMaxHood
            if (rightShift) {
                rightShift = false;
                setpoint -= magnitude;
            }
        }

        if (joystick.getRawAxis(0) < -deadZone) { // && !shooterSubsystem.getSensors().reachedMinHood
            if (leftShift) {
                leftShift = false;
                setpoint += magnitude;
            }
        }
        Logger.recordOutput("preClampTurretSetpoint",turretSetpoint);
        turretSetpoint = MathUtil.clamp(
                turretSetpoint,
                shooterSubsystem.getSensors().turretMinAngle,
                shooterSubsystem.getSensors().turretMaxAngle
        );
        turretPIDController.setSetpoint(turretSetpoint);
        Logger.recordOutput("turretSetpoint", turretPIDController.getSetpoint());
        Logger.recordOutput("turretRotation", shooterSubsystem.getTurretAngleinDegrees().in(Degrees));

        double output = turretPIDController.calculate(
                shooterSubsystem.getTurretAngleinDegrees().in(Degrees),
                turretSetpoint);
        Logger.recordOutput("turretOutput", output);
        shooterSubsystem.setTurretPower(output);

        if (joystick.getRawAxis(0) >= -deadZone && joystick.getRawAxis(0) <= deadZone) {
            turretSetpoint = shooterSubsystem.getTurretAngleinDegrees().in(Degrees);
            shooterSubsystem.setTurretPower(0);
            rightTurretShift = true;
            leftTurretShift = true;
        }
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


