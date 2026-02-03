package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.PhotonCameraObject;
import frc.robot.subsystem.TankDrive;

import java.util.List;
import java.util.Optional;

public class DriveToTag extends Command {
    public final TankDrive drive;
    public final PhotonCameraObject camera;


    public DriveToTag(TankDrive drive, PhotonCameraObject camera, String pathName) {
        this.drive = drive;
        this.camera = camera;

        addRequirements(drive);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
