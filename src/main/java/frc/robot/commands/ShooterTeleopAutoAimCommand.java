package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
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

    private InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();


    boolean isFlywheelOn = false;

    private Pose2d lockedPose;

    public double kP = 2.5 / 1500.0;
    public double kI = 0.00015;
    public double kD = 0.4 / 14000;
    public double IZone = 1000;

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

    public ShooterTeleopAutoAimCommand(RobotContainer robot, Joystick joystick) {
        this.joystick = joystick;
        this.shooterSubsystem = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();

        shooterPIDController.setTolerance(100);
        shooterMap.put(1.975, 3400.0);
        shooterMap.put(3.258, 3892.0);
        shooterMap.put(4.75, 3986.0);
        shooterMap.put(4.61,4902.0);


        hoodMap.put(1.975, 177.2);
        hoodMap.put(3.258, 182.5);
        hoodMap.put(4.75, 188.4);
        hoodMap.put(4.61, 188.11);

        addRequirements(shooterSubsystem);



    }

    @Override
    public void initialize() {
        turretSetpoint = shooterSubsystem.getTurretAngle().in(Degrees);
        hoodSetpoint = shooterSubsystem.getHoodAngleFromThroughbore().in(Degrees);
        shooterSetpoint = 4300;

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

            if (isFlywheelOn) {
                lockedPose = drive.getPose();
            }
        }

        Pose2d pose = (lockedPose != null) ? lockedPose : drive.getPose();

        double distance = pose.getTranslation().plus(new Translation2d(Inches.of(2.172), Inches.of(-8.4375))).getDistance(getHunPosition());



        Rotation2d targetTurretAngle = getHunPosition().minus(pose.getTranslation()).getAngle();

        Rotation2d robotHeading = pose.getRotation();

        Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading);


        double currTurretAngle = shooterSubsystem.getSensors().turretRelativeAngle.in(Degrees);


        double desiredAngle = desiredTurret.getDegrees();



        Logger.recordOutput("LockedDistance", distance);



        if (isFlywheelOn) {
          // shooterSetpoint = calculateShooterSpeed(distance);
         //  hoodSetpoint = calcHoodAngle(distance);
         //  turretSetpoint = desiredAngle;

        }

        if (joystick.getRawButton(3)) {
            shooterSetpoint -= 2;
        }
        if (joystick.getRawButton(4)) {
            shooterSetpoint += 2;
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
            shooterSubsystem.setRampPower(0.7);



        } else {
            shooterSubsystem.setFlywheelPower(0);
            shooterSubsystem.setTransitionPower(0);
            shooterSubsystem.setRampPower(0);

        }

        shooterSubsystem.getSensors().atShooterSpeed = shooterPIDController.atSetpoint();

        if (joystick.getRawAxis(3) > 0.3) {
            shooterSubsystem.setSpindexerPower(1);
        } else if (joystick.getRawAxis(2) > 0.3) {
            shooterSubsystem.setSpindexerPower(-1);
            shooterSubsystem.setTransitionPower(-0.6);
            shooterSubsystem.setRampPower(-0.6);

        } else {
            shooterSubsystem.setSpindexerPower(0);
        }

        if (joystick.getRawAxis(0) > deadZone) {
            turretSetpoint -= 5;
        }
        if (joystick.getRawAxis(0) < -deadZone) {
            turretSetpoint += 5;
        }

        turretSetpoint = MathUtil.clamp(turretSetpoint, shooterSubsystem.getSensors().turretMinAngle, shooterSubsystem.getSensors().turretMaxAngle);

        turretPIDController.setSetpoint(turretSetpoint);

        double turretOutput = turretPIDController.calculate(shooterSubsystem.getTurretAngle().in(Degrees), turretSetpoint);

      // double turretOutput = turretPIDController.calculate(currTurretAngle, turretSetpoint);



        if (turretOutput > 0.4) turretOutput = 0.4;



        shooterSubsystem.setTurretPower(turretOutput);


        if (!isFlywheelOn) {
            if (joystick.getRawAxis(5) > deadZone) {
                hoodSetpoint -= 2 / 8.0;
            }
            if (joystick.getRawAxis(5) < -deadZone) {
                hoodSetpoint += 2 / 8.0;
            }
        } else {
           // hoodSetpoint = calcHoodAngle(distance);
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

    private Translation2d getHunPosition() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isPresent()) {
            isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }





        return isRed
                ? new Translation2d(12.286869, 4.034)
                : new Translation2d(4.624, 4.034);
    }

    private double calculateShooterSpeed(double distance) {
        return 3040.48652 + 219.83512* distance;

       // return shooterMap.get(distance);
    }

    private double calcHoodAngle(double distance) {
        return 169.17252 + 4.07866 * distance;
        // return hoodMap.get(distance);

    }
}
