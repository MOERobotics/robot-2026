package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.sin;

public class ShooterTeleopAutoAimCommand extends Command {

    public final ShooterSubsystem shooter;
    public final Joystick joystick;
    private final SwerveDriveSubsystem drive;


    boolean isFlywheelOn = false;

    public double kP = 1.8 / 1500.0;
    public double kI = 0.0005;
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

    Translation2d hubPosition = new Translation2d();



    public ShooterTeleopAutoAimCommand(RobotContainer robot, Joystick joystick) {
        this.joystick = joystick;
        this.shooter = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();

        shooterPIDController.setTolerance(100);


        addRequirements(shooter);



    }

    @Override
    public void initialize() {

        turretSetpoint = shooter.getSensors().turretRelativeAngle.in(Degrees);
        hoodSetpoint = shooter.getHoodAngleFromThroughbore().in(Degrees);
        shooterSetpoint = 3000;
        isFlywheelOn = false;

        hoodPIDController.setSetpoint(hoodSetpoint);
        turretPIDController.setSetpoint(turretSetpoint);

        shooterPIDController.reset();
        shooterPIDController.setIZone(IZone);
        shooterPIDController.setIntegratorRange(-.05, .05);

        hubPosition = shooter.getHubPosition();


    }

    @Override
    public void execute() {



        if (joystick.getRawButtonPressed(2)) {
            isFlywheelOn = !isFlywheelOn;
            shooterPIDController.reset();
        }

        Pose2d pose = drive.getPose();

        Translation2d turretOffset = ShooterSubsystem.turretOffset.rotateBy(pose.getRotation());

        shooter.getSensors().turretOffset = turretOffset;

        

        // NOTE DUMMY NUMBER ADDITION - nvm
        currDistance = shooter.getDistance(turretPosition, hubPosition);//- 20;

        turretPosition = shooter.getTurretPosition(pose,turretOffset);

        shooter.getSensors().turretPosition = turretPosition;

        if (joystick.getRawButtonPressed(1)) {

            distance =  shooter.getDistance(turretPosition, hubPosition);// - 20;

            hoodSetpoint = shooter.calcHoodAngle(distance);
            shooterSetpoint = shooter.calculateShooterSpeed(distance);

// turretSetpoint = desiredAngle;
//x, y are field coordinates of CoR
// z is field centric angle of robot
// a is distance from CoR to Center of turrent (front-back)
// b is distance from CoR to CoT (left-right)
// theta = z+180 where z is robot field angle
// phi = atan2(ty-(y+a*sin(theta)-b*cos(theta),tx-(x+a*cos(theta)+b*sin(theta)) -theta

            Logger.recordOutput("Distance", distance);


        }

        Logger.recordOutput("TurretPositionIThink",
                new Pose2d(
                        turretPosition,
                        new Rotation2d(pose.getRotation().plus(Rotation2d.kPi).getMeasure().plus(shooter.getSensors().turretRelativeAngle))
                )
        );
        Logger.recordOutput("TargetTurretAngle",  hubPosition.minus(turretPosition).getAngle());
        Logger.recordOutput("Kevin's method1", hubPosition.minus(pose.plus(new Transform2d(turretOffset, Rotation2d.kZero)).getTranslation()).getAngle());
        Logger.recordOutput("Kevin's method2", hubPosition.minus(pose.plus(new Transform2d(turretOffset, pose.getRotation())).getTranslation()).getAngle());
        Logger.recordOutput("Distance", distance);



        int currentPOV = joystick.getPOV();
        distance = shooter.getDistance(turretPosition, hubPosition);
        desiredTurretAngle = shooter.getTurretAimAngle(pose.getRotation(), turretPosition, hubPosition) +180;

        if (currentPOV != -1) {
            switch (currentPOV) {
                case 90:






                    //x, y are field coordinates of CoR
// z is field centric angle of robot
// a is distance from CoR to Center of turret (front-back)
// b is distance from CoR to CoT (left-right)
// theta = z+180 where z is robot field angle
// phi = atan2(ty-(y+a*sin(theta)-b*cos(theta),tx-(x+a*cos(theta)+b*sin(theta)) -theta

                    turretSetpoint = desiredTurretAngle;
                    hoodSetpoint = shooter.calcHoodAngle(distance);
                    shooterSetpoint = shooter.calculateShooterSpeed(distance);

                    break;
            }





        }

        double x = pose.getX();
        double y = pose.getY();
        double theta = (pose.getRotation().getDegrees() +180) % 360;
        double a = Inches.of(Math.abs(-2.172)).in(Meters);
        double b =  Inches.of(Math.abs(8.4375)).in(Meters);;

        double phi = Math.atan2(
                // y1 = bubY - (y + a*sin(theta) - b*cos(theta))
                // x1 = hubX - (x + a*cos(theta) + b*sin(theta)) - theta(in degrees)
                hubPosition.getY()-(y+a*Math.sin(Degrees.of(theta).in(Radian))-b*Math.cos(Degrees.of(theta).in(Radians))),
                hubPosition.getX()-(x+a*Math.cos(Degrees.of(theta).in(Radian))+b*sin(Degrees.of(theta).in(Radians))))  -Degrees.of(theta).in(Radian);



        Logger.recordOutput("Phi", phi);


//        Logger.recordOutput("ConstantTurretAngle", shooter.getTurretAimAngle(pose.getRotation(), turretPosition, hubPosition));



        Logger.recordOutput("HoodSetpoint", hoodSetpoint);

        currDistance = shooter.getDistance(turretPosition, shooter.getHubPosition());


        Logger.recordOutput("CurrDistance", currDistance);



        if (joystick.getRawButton(3)) {
            shooterSetpoint -=5;
        }
        if (joystick.getRawButton(4)) {
            shooterSetpoint += 5;
        }

        shooterPIDController.setSetpoint(shooterSetpoint);

        Logger.recordOutput("FlywheelSetpoint", shooterSetpoint);
        Logger.recordOutput("TurretSetpoint", turretSetpoint);




        if (isFlywheelOn) {
            double currentRPM = shooter.getFlywheelSpeed().in(RPM);
            Logger.recordOutput("FlywheelI", shooterPIDController.getAccumulatedError() * kI);
            double output = shooterPIDController.calculate(currentRPM, shooterSetpoint);

            double feedforward = shooter.feedForwardCalc(shooterSetpoint);
            double outputMax = 1 - feedforward;

            if (output > outputMax) output = outputMax;
            if (output < 0) output = 0;

            shooter.setFlywheelPower(feedforward + output);
            shooter.setTransitionPower(0.7);
            shooter.setRampPower(0.6);



        } else {
            shooter.setFlywheelPower(0);
            shooter.setTransitionPower(0);
            shooter.setRampPower(0);

        }

        shooter.getSensors().atShooterSpeed = shooterPIDController.atSetpoint();

        if (joystick.getRawAxis(3) > 0.3) {
            shooter.setSpindexerPower(0.75);
        } else if (joystick.getRawAxis(2) > 0.3) {
            shooter.setSpindexerPower(-1);
            shooter.setTransitionPower(-0.6);
            shooter.setRampPower(-0.6);

        } else {
            shooter.setSpindexerPower(0);
        }

        if (joystick.getRawAxis(0) > deadZone) {
            turretSetpoint -= 0.7;
        }
        if (joystick.getRawAxis(0) < -deadZone) {
            turretSetpoint += 0.7;
        }

        turretSetpoint = MathUtil.clamp(turretSetpoint, shooter.getSensors().turretMinAngle, shooter.getSensors().turretMaxAngle);

        turretPIDController.setSetpoint(turretSetpoint);

        double turretOutput = turretPIDController.calculate(shooter.getSensors().turretRelativeAngle.in(Degrees));




        if (turretOutput > 0.6) turretOutput = 0.6;



        shooter.setTurretPower(turretOutput);

        if (joystick.getRawAxis(5) > deadZone) {
            hoodSetpoint -= 5.5 / 10.0;
        }
        if (joystick.getRawAxis(5) < -deadZone) {
            hoodSetpoint += 5.5 / 10.0;
        }



        hoodSetpoint = MathUtil.clamp(hoodSetpoint, shooter.getSensors().hoodMinAngle, shooter.getSensors().hoodMaxAngle);

        hoodPIDController.setSetpoint(hoodSetpoint);

        double hoodOutput = hoodPIDController.calculate(shooter.getHoodAngleFromThroughbore().in(Degrees));

        if (hoodOutput > 0.4) hoodOutput = 0.4;

        shooter.setHoodPower(hoodOutput);






    }


    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelPower(0);
        shooter.setRampPower(0);
        shooter.setTurretPower(0);
        shooter.setHoodPower(0);
        shooter.stopFeeding();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
