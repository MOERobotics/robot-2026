package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import static java.lang.Math.sin;

public class ShooterTeleopAutoAimCommand extends Command {

    public final ShooterSubsystem shooterSubsystem;
    public final Joystick joystick;
    private final SwerveDriveSubsystem drive;


    boolean isFlywheelOn = false;

    public double kP = 1.8 / 1500.0;
    public double kI = 0.00015;
    public double kD = 0.8 / 14000;
    public double IZone = 500;

    public double hoodKP = 0.056;
    public double hoodKI = 0;
    public double hoodKD = 0;

    public double deadZone = 0.4;

    public double turretKP = 0.055;
    public double turretKI = 0;
    public double turretKD = 0;

    PIDController shooterPIDController = new PIDController(kP, kI, kD);
    PIDController hoodPIDController = new PIDController(hoodKP, hoodKI, hoodKD);
    PIDController turretPIDController = new PIDController(turretKP, turretKI, turretKD);

    public double turretSetpoint, hoodSetpoint, shooterSetpoint;

    double currDistance=0;
    double distance=0;

    double desiredTurretAngle=0;

    Translation2d turretPosition = new Translation2d();


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

        Pose2d pose = drive.getPose();


        Translation2d turretOffset = new Translation2d(Inches.of(-2.172), Inches.of(8.4375)).rotateBy(pose.getRotation());
        // NOTE DUMMY NUMBER ADDITION
        currDistance = Meters.of(pose.getTranslation().plus(turretOffset).getDistance(getHubPosition())).in(Inches)-10;
        turretPosition = pose.getTranslation().plus(turretOffset);






        if (joystick.getRawButtonPressed(1)) {
             distance = Meters.of(turretPosition.getDistance(getHubPosition())).in(Inches);
            Rotation2d targetTurretAngle = getHubPosition().minus(turretPosition).getAngle();

            Rotation2d robotHeading = pose.getRotation();

            Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading);

            //desiredTurretAngle = desiredTurret.getDegrees();


            shooterSetpoint = calculateShooterSpeed(distance);
           // turretSetpoint = desiredAngle;
          hoodSetpoint = calcHoodAngle(distance);
//x, y are field coordinates of CoR
// z is field centric angle of robot
// a is distance from CoR to Center of turrent (front-back)
// b is distance from CoR to CoT (left-right)
// theta = z+180 where z is robot field angle
// phi = atan2(ty-(y+a*sin(theta)-b*cos(theta),tx-(x+a*cos(theta)+b*sin(theta)) -theta

            Logger.recordOutput("Distance", distance);


        }


        Logger.recordOutput("turretPosition", turretPosition);


        int currentPOV = joystick.getPOV();

        if (currentPOV != -1) {
            switch (currentPOV) {
                case 90:
                    Logger.recordOutput("TurretPositionIThink", new Pose2d(turretPosition, pose.getRotation()));
                    distance = Meters.of(turretPosition.getDistance(getHubPosition())).in(Inches);
                    Rotation2d targetTurretAngle = getHubPosition().minus(turretPosition).getAngle();

                    Rotation2d robotHeading = pose.getRotation();

                    Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading).plus(Rotation2d.k180deg);

                    desiredTurretAngle = desiredTurret.getDegrees();
                    while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
                    while (desiredTurretAngle < -180) desiredTurretAngle += 360;


                    shooterSetpoint = calculateShooterSpeed(distance);

                    Logger.recordOutput("TargetTurretAngle", targetTurretAngle);
                    Logger.recordOutput("Kevin's method1", getHubPosition().minus(pose.plus(new Transform2d(turretOffset, Rotation2d.kZero)).getTranslation()).getAngle());
                    Logger.recordOutput("Kevin's method2", getHubPosition().minus(pose.plus(new Transform2d(turretOffset, pose.getRotation())).getTranslation()).getAngle());
                    Logger.recordOutput("Distance", distance);


                    //x, y are field coordinates of CoR
// z is field centric angle of robot
// a is distance from CoR to Center of turret (front-back)
// b is distance from CoR to CoT (left-right)
// theta = z+180 where z is robot field angle
// phi = atan2(ty-(y+a*sin(theta)-b*cos(theta),tx-(x+a*cos(theta)+b*sin(theta)) -theta


                    turretSetpoint = desiredTurretAngle;
                    hoodSetpoint = calcHoodAngle(currDistance);
                    break;
            }





        }

        double x = pose.getX();
        double y = pose.getY();
        double theta = (pose.getRotation().getDegrees() +180) % 360;
        double a = Inches.of(Math.abs(-2.172)).in(Meters);
        double b =  Inches.of(Math.abs(8.4375)).in(Meters);;

        double phi = Math.atan2(
                getHubPosition().getY()-(y+a*Math.sin(Degrees.of(theta).in(Radian))-b*Math.cos(Degrees.of(theta).in(Radians))),
                getHubPosition().getX()-(x+a*Math.cos(Degrees.of(theta).in(Radian))+b*sin(Degrees.of(theta).in(Radians))))  -Degrees.of(theta).in(Radian);


        Logger.recordOutput("BlueTurretAngle", phi);


        Logger.recordOutput("desiredTurretAngle", desiredTurretAngle);



        Logger.recordOutput("HoodSetpoint", hoodSetpoint);



        Logger.recordOutput("CurrDistance", currDistance);

        // logging angle constantly

        Rotation2d targetTurretAngle = getHubPosition().minus(turretPosition).getAngle();

        Rotation2d robotHeading = pose.getRotation();

        Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading).plus(Rotation2d.k180deg);

        desiredTurretAngle = desiredTurret.getDegrees();
        while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
        while (desiredTurretAngle < -180) desiredTurretAngle += 360;

        //


        if (joystick.getRawButton(3)) {
            shooterSetpoint -=5;
        }
        if (joystick.getRawButton(4)) {
            shooterSetpoint += 5;
        }

        shooterPIDController.setSetpoint(shooterSetpoint);

        Logger.recordOutput("FlywheelSetpoint", shooterSetpoint);
        Logger.recordOutput("TurretSetpoint", turretSetpoint);

        Logger.recordOutput("FlywheelSpeed", shooterSubsystem.getFlywheelSpeed().in(RPM));

        Logger.recordOutput("turretOffset", turretOffset);


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
            turretSetpoint -= 0.3;
        }
        if (joystick.getRawAxis(0) < -deadZone) {
            turretSetpoint += 0.3;
        }

        turretSetpoint = MathUtil.clamp(turretSetpoint, shooterSubsystem.getSensors().turretMinAngle, shooterSubsystem.getSensors().turretMaxAngle);

        turretPIDController.setSetpoint(turretSetpoint);

        double turretOutput = turretPIDController.calculate(shooterSubsystem.getSensors().turretRelativeAngle.in(Degrees));

      //double turretOutput = turretPIDController.calculate(currTurretAngle, turretSetpoint);



        if (turretOutput > 0.6) turretOutput = 0.6;



        shooterSubsystem.setTurretPower(turretOutput);

        if (joystick.getRawAxis(5) > deadZone) {
            hoodSetpoint -= 1 / 10.0;
        }
        if (joystick.getRawAxis(5) < -deadZone) {
            hoodSetpoint += 1 / 10.0;
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
