package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class HoodCommand extends Command {

    private final ShooterSubsystem shooter;
    private final SwerveDriveSubsystem drive;
    private final Joystick joystick;

    private final PIDController turretPID = new PIDController(4.5, 0.0, 0.1);

    private final PIDController hoodPID = new PIDController(0.035, 0.0, 0.0);

    private boolean autoMode = false;
    private Translation2d currentTarget = null;

    public HoodCommand(RobotContainer robot, Joystick joystick) {

        this.shooter = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();
        this.joystick = joystick;

        turretPID.setTolerance(Math.toRadians(1.5));
        hoodPID.setTolerance(1.0);
        addRequirements(shooter);
    }

    @Override
    public void execute() {


        double turretAxis = MathUtil.applyDeadband(joystick.getRawAxis(2), 0.15);

        double hoodAxis = MathUtil.applyDeadband(joystick.getRawAxis(1), 0.15);

        if (Math.abs(turretAxis) > 0.05 || Math.abs(hoodAxis) > 0.05) {

            autoMode = false;
            shooter.setTurretPower(turretAxis);
            shooter.setHoodPower(hoodAxis);
            return;
        }


        if (joystick.getRawButtonPressed(6)) {
            currentTarget = getHubPosition();
            autoMode = true;
        }

        if (joystick.getRawButtonPressed(7)) {
            currentTarget = getDepotPosition();
            autoMode = true;
        }

        if (joystick.getRawButtonPressed(8)) {
            currentTarget = getOutpostPosition();
            autoMode = true;
        }


        if (autoMode && currentTarget != null) {

            Pose2d pose = drive.getPose();

            Rotation2d angleToTarget = new Rotation2d(currentTarget.getX() - pose.getX(), currentTarget.getY() - pose.getY());

            Rotation2d robotHeading = pose.getRotation();

            Rotation2d desiredTurretAngle = angleToTarget.minus(robotHeading);


            double desired = MathUtil.clamp(desiredTurretAngle.getRadians(), Math.toRadians(-90), Math.toRadians(90));

            double current = Math.toRadians(shooter.getTurretAngleinDegrees().in(Degrees));

            double turretOutput = turretPID.calculate(current, desired);

            shooter.setTurretPower(turretOutput);

            double distance = pose.getTranslation().getDistance(currentTarget);

            double hoodAngle = calculateHoodAngle(distance);

            double currentHoodAngle = shooter.getHoodAngleinDegrees().in(Degrees);

            double hoodOutput = hoodPID.calculate(currentHoodAngle, hoodAngle);




            if (hoodOutput > 0 && shooter.reachedHoodMax())
                hoodOutput = 0;

            if (hoodOutput < 0 && shooter.reachedHoodMin())
                hoodOutput = 0;

            shooter.setHoodPower(hoodOutput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTurretPower(0);
        shooter.setHoodPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    private double calculateHoodAngle(double distance) {

        double targetHeight = 2.5;
        double rHeight = 0.5;
        double heightDiff = targetHeight - rHeight;

        double radius = 0.05;
        double velocity = shooter.getFlywheelSpeed().in(RPM) * (radius) ;

        double g = 9.81;

        double num = Math.pow(velocity, 4) - g * (g * distance * distance + 2 * heightDiff * velocity * velocity);

        if (num < 0)
            return 0;

        double angle = Math.atan((velocity * velocity - Math.sqrt(num)) / (g * distance));

        return Math.toDegrees(angle);
    }


    public Translation2d getOutpostPosition() {

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return new Translation2d(0, 0);
            } else {
                return new Translation2d(0, 0);
            }

        } else {
            return new Translation2d(0, 0);
        }
    }

    public Translation2d getHubPosition() {

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return new Translation2d(0, 0);
            } else {
                return new Translation2d(0, 0);
            }

        } else {
            return new Translation2d(0, 0);
        }
    }


    public Translation2d getDepotPosition() {

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return new Translation2d(0, 0);
            } else {
                return new Translation2d(0, 0);
            }

        } else {
            return new Translation2d(0, 0);
        }
    }

}