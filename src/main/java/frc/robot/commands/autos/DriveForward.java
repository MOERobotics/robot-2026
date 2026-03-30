package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autos;
import frc.robot.commands.PathsFollower;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.Logger;


public class DriveForward {

    public static Autos.CommandAndPose drive15Ft(RobotContainer robot) {
        return driveForward(robot, "DriveForward15Ft");
    }


    public static Autos.CommandAndPose driveForward(
            RobotContainer robot,
            String path1) {


        PathsFollower plannerPath1 = new PathsFollower(path1);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();
        Logger.recordOutput("forwardPath", plannerPath1.path.getPathPoses().toArray(Pose2d[]::new));

        Command auto = Commands.sequence(
                plannerPath1,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())
        );


        return new Autos.CommandAndPose(auto, startingPose);
    }

}
