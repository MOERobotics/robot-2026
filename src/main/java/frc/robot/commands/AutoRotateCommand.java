package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class AutoRotateCommand  extends Command {

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Joystick joystick;

    private  PIDController rotatePID = new PIDController(1,0,0);


    private double targetAngle =0;
    public AutoRotateCommand(RobotContainer robot, Joystick joystick) {
        this.swerveDriveSubsystem = robot.getRobotSwerveDrive();
        this.joystick = joystick;
        rotatePID.setTolerance(Math.toRadians(1));
        rotatePID.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {

        int pov = joystick.getPOV();

        if (pov != -1) {
            targetAngle = switch (pov) {
                case 0 -> 90.0;
                case 90 -> 0.0;
                case 180 -> 180.0;
                case 270 -> 270.0;
                default -> -1;
            };
        }

        if (targetAngle == -1) {
            swerveDriveSubsystem.robotDrive(new ChassisSpeeds(0,0,0), true);
            return;
        }

        double currAngle = swerveDriveSubsystem.getPose().getRotation().getRadians();

        double targetRad = Math.toRadians(targetAngle);

        double output = rotatePID.calculate(currAngle, targetRad);

        swerveDriveSubsystem.robotDrive(new ChassisSpeeds(0, 0, 3*output), true);

        Logger.recordOutput("Rotate/Target", targetAngle);
        Logger.recordOutput("Rotate/Output", output);
        Logger.recordOutput("Rotate/Omega", swerveDriveSubsystem.getChassisSpeed().omegaRadiansPerSecond);


    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.robotDrive(new ChassisSpeeds(0,0,0), false);
    }

    @Override
    public boolean isFinished() {
        return rotatePID.atSetpoint();
    }
}