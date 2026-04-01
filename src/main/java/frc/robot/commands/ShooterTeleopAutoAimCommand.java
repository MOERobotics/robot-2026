package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ShooterTeleopAutoAimCommand extends Command {

    public final ShooterSubsystem shooterSubsystem;
    public final Joystick joystick;
    private final SwerveDriveSubsystem drive;


    boolean isFlywheelOn = false;

    private Pose2d lockedPose;

    public double kP = 1.8 / 1500.0;
    public double kI = 0.00015;
    public double kD = 0.8 / 14000;
    public double IZone = 500;

    public double hoodKP = 0.056;
    public double hoodKI = 0;
    public double hoodKD = 0;

    public double deadZone = 0.4;

    public double turretKP = 0.0055;
    public double turretKI = 0;
    public double turretKD = 0;

    PIDController shooterPIDController = new PIDController(kP, kI, kD);
    PIDController hoodPIDController = new PIDController(hoodKP, hoodKI, hoodKD);
    PIDController turretPIDController = new PIDController(turretKP, turretKI, turretKD);

    public double turretSetpoint, hoodSetpoint, shooterSetpoint;

    double currDistance=0;
    double distance=0;

    public ShooterTeleopAutoAimCommand(RobotContainer robot, Joystick joystick) {
        this.joystick = joystick;
        this.shooterSubsystem = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();

        shooterPIDController.setTolerance(100);


        addRequirements(shooterSubsystem);



    }

    @Override
    public void initialize() {
        turretSetpoint = shooterSubsystem.getSensors().turretRelativeAngle.in(Degrees);
        hoodSetpoint = shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees);
        shooterSetpoint = 3000;

        hoodPIDController.setSetpoint(hoodSetpoint);
        turretPIDController.setSetpoint(turretSetpoint);

        shooterPIDController.reset();
        shooterPIDController.setIZone(IZone);
        shooterPIDController.setIntegratorRange(-.05, .05);
    }

    @Override
    public void execute() {

        if (joystick.getRawButtonPressed(2)) {
            isFlywheelOn = !isFlywheelOn;
            shooterPIDController.reset();


        }

        Translation2d turretOffset = new Translation2d(Inches.of(2.172), Inches.of(-8.4375)).rotateBy(drive.getPose().getRotation());
        currDistance = Meters.of(drive.getPose().getTranslation().plus(turretOffset).getDistance(getHubPosition())).in(Inches);





        if (joystick.getRawButtonPressed(1)) {

            lockedPose = drive.getPose();
            Pose2d pose =lockedPose;

            Translation2d lockedOffset = new Translation2d(
                    Inches.of(2.172),
                    Inches.of(-8.4375)).rotateBy(pose.getRotation());

            Translation2d turretPosition = pose.getTranslation().plus(turretOffset);



            distance = Meters.of(turretPosition.getDistance(getHubPosition())).in(Inches);


            Rotation2d targetTurretAngle = getHubPosition().minus(turretPosition).getAngle();

            Rotation2d robotHeading = pose.getRotation();

            Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading);

            double desiredAngle = desiredTurret.getDegrees();


            shooterSetpoint = calculateShooterSpeed(currDistance);

            Logger.recordOutput("calcHoodSetpoint", hoodSetpoint);
            Logger.recordOutput("calcShooterSetpoint", shooterSetpoint);


            //turretSetpoint = desiredAngle-20;
            hoodSetpoint = calcHoodAngle(currDistance);


        }



        Logger.recordOutput("LockedDistance", distance);

        Logger.recordOutput("HoodSetpoint", hoodSetpoint);



        Logger.recordOutput("CurrDistance", currDistance);




        if (joystick.getRawButton(3)) {
            shooterSetpoint -=5;
        }
        if (joystick.getRawButton(4)) {
            shooterSetpoint += 5;
        }

        shooterPIDController.setSetpoint(shooterSetpoint);

        Logger.recordOutput("FlywheelSetpoint", shooterSetpoint);
        Logger.recordOutput("FlywheelSpeed", shooterSubsystem.getFlywheelSpeed().in(RPM));

        if (isFlywheelOn) {
            double currentRPM = shooterSubsystem.getFlywheelSpeed().in(RPM);

            double output = shooterPIDController.calculate(currentRPM, shooterSetpoint);

            double feedforward = shooterSubsystem.feedForwardCalc(shooterSetpoint);
            double outputMax = 1 - feedforward;

            if (output > outputMax) output = outputMax;
            if (output < 0) output = 0;

            shooterSubsystem.setFlywheelPower(feedforward + output);
            shooterSubsystem.setTransitionPower(0.7);
            shooterSubsystem.setRampPower(0.6);



        } else {
            shooterSubsystem.setFlywheelPower(0);
            shooterSubsystem.setTransitionPower(0);
            shooterSubsystem.setRampPower(0);

        }

        shooterSubsystem.getSensors().atShooterSpeed = shooterPIDController.atSetpoint();

        if (joystick.getRawAxis(3) > 0.3) {
            shooterSubsystem.setSpindexerPower(0.75);
        } else if (joystick.getRawAxis(2) > 0.3) {
            shooterSubsystem.setSpindexerPower(-1);
            shooterSubsystem.setTransitionPower(-0.6);
            shooterSubsystem.setRampPower(-0.6);

        } else {
            shooterSubsystem.setSpindexerPower(0);
        }

        if (joystick.getRawAxis(0) > deadZone) {
            turretSetpoint -= 0.2;
        }
        if (joystick.getRawAxis(0) < -deadZone) {
            turretSetpoint += 0.2;
        }

        turretSetpoint = MathUtil.clamp(turretSetpoint, shooterSubsystem.getSensors().turretMinAngle, shooterSubsystem.getSensors().turretMaxAngle);

        turretPIDController.setSetpoint(turretSetpoint);

        double turretOutput = turretPIDController.calculate(shooterSubsystem.getSensors().turretRelativeAngle.in(Degrees));

      // double turretOutput = turretPIDController.calculate(currTurretAngle, turretSetpoint);



        if (turretOutput > 0.4) turretOutput = 0.4;



        shooterSubsystem.setTurretPower(turretOutput);

        if (joystick.getRawAxis(5) > deadZone) {
            hoodSetpoint -= 1 / 15.0;
        }
        if (joystick.getRawAxis(5) < -deadZone) {
            hoodSetpoint += 1 / 15.0;
        }



        hoodSetpoint = MathUtil.clamp(hoodSetpoint, shooterSubsystem.getSensors().hoodMinAngle, shooterSubsystem.getSensors().hoodMaxAngle);

        hoodPIDController.setSetpoint(hoodSetpoint);

        double hoodOutput = hoodPIDController.calculate(shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees));

        if (hoodOutput > 0.4) hoodOutput = 0.4;

        shooterSubsystem.setHoodPower(hoodOutput);




    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelPower(0);
        shooterSubsystem.setRampPower(0);
        shooterSubsystem.setTurretPower(0);
        shooterSubsystem.setHoodPower(0);
        shooterSubsystem.stopFeeding();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private Translation2d getHubPosition() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isPresent()) {
            isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

        return isRed
                ? new Translation2d(12.286869, 4.034)
                : new Translation2d(4.624, 4.034);
    }

    private double calculateShooterSpeed(double distance) {
        return 6.18842*distance+2429.32725;
       // return shooterMap.get(distance);
    }

    private double calcHoodAngle(double distance) {
        return 0.000586636*Math.pow(distance, 2) - (0.0564512*distance) + 187.01223;

        // return hoodMap.get(distance);

    }
}
