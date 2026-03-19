package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class DriveTeleopCommand extends Command {

    private final SwerveDriveSubsystem drive;
    private final Joystick joystick;

    private final double num = 2;

    double avgX =0;

    double avgY =0;
    double avgRotation =0;



    public DriveTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.drive = robot.getRobotSwerveDrive();
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override public void initialize(){
        avgX = joystick.getRawAxis(0);
        avgY = joystick.getRawAxis(1);
        avgX = joystick.getRawAxis(2);

    }
    @Override
    public void execute() {


        // new averaging thing :]

        avgX = (avgX * ((num-1)/num)) + (joystick.getRawAxis(1)/num);
        avgY= (avgY * ((num-1)/num)) + (joystick.getRawAxis(0)/num);
        avgRotation = (avgRotation * ((num-1)/num)) + (joystick.getRawAxis(2)/num);

        Logger.recordOutput("Avg X", avgX);
        Logger.recordOutput("Avg Y", avgY);
        Logger.recordOutput("Avg Rotation", avgRotation);

        ChassisSpeeds speeds = new ChassisSpeeds(
                MathUtil.applyDeadband(avgX * -1, 0.06),
                MathUtil.applyDeadband(avgY * -1, 0.06),
                MathUtil.applyDeadband(avgRotation * -1, 0.06)
        );


        drive.robotDrive(speeds, false);


    }

    @Override
    public void end(boolean interrupted) {
        drive.robotDrive(new ChassisSpeeds(), false);
    }
}