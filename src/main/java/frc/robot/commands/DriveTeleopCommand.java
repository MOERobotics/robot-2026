package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;

public class DriveTeleopCommand extends Command {

    private final SwerveDriveSubsystem drive;
    private final Joystick joystick;

    public DriveTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.drive = robot.getRobotSwerveDrive();
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override
    public void execute() {

        ChassisSpeeds speeds = new ChassisSpeeds(
                MathUtil.applyDeadband(joystick.getRawAxis(1) * -1, 0.06),
                MathUtil.applyDeadband(joystick.getRawAxis(0) * -1, 0.06),
                MathUtil.applyDeadband(joystick.getRawAxis(2) * -1, 0.06)
        );

        drive.robotDrive(speeds, true);

    }

    @Override
    public void end(boolean interrupted) {
        drive.robotDrive(new ChassisSpeeds(), false);
    }
}