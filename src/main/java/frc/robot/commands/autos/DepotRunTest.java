package frc.robot.commands.autos;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.container.RobotContainer;

import java.nio.file.Path;

public class DepotRunTest {

    public static Autos.CommandAndPose getAuto(RobotContainer robot) {

        PathsFollower ALT_DEPOT = new PathsFollower("ALT-Depot");

        //Flip Pose if needed
        Pose2d startingPoseBlue = ALT_DEPOT.path.getStartingHolonomicPose().get();
        final Pose2d startingPose;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            startingPose = FlippingUtil.flipFieldPose(startingPoseBlue);
        } else {
            startingPose = startingPoseBlue;
        }


        Command autoCommand = Commands.sequence(
                Commands.deadline(
                       ALT_DEPOT),
                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()
                ),
                Commands.runOnce(
                        () -> System.err.println("\n\n\n\n\n\n\n\nDone!\n\n\n\n\n\n\n\n\n\n\n")
                ));



        return new Autos.CommandAndPose(autoCommand, startingPoseBlue);
    }
}