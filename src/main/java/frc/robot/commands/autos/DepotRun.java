package frc.robot.commands.autos;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.PathsFollower;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.container.RobotContainer;
import frc.robot.AutosChooser.CommandAndPose;

public class DepotRun {

    public static CommandAndPose getAuto(RobotContainer robot) {

        Command autoCommand = Commands.sequence(
                Commands.deadline(
                        new PathsFollower("ALT-Depot"),
                        new FuelCollectorAutoCommand(robot,true, true,"in")
                ).withTimeout(3),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),
                Commands.runOnce(
                        () -> System.err.println("\n\n\n\n\n\n\n\nDone!\n\n\n\n\n\n\n\n\n\n\n")
                ));


        Pose2d startPose = new PathsFollower("ALT-Depot").path.getStartingHolonomicPose().get();

        return new CommandAndPose(autoCommand, startPose);
    }
}