package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.ClimberAutoCommand;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.container.RobotContainer;

public class DepotRun{

    public static Autos.CommandAndPose getAuto(RobotContainer robot) {

        Command autoCommand =  Commands.sequence(
                Commands.deadline(
                        new PathsFollower("ALT-Depot"),
                        new FuelCollectorAutoCommand(robot,true, true,"stop")
                ).withTimeout(3),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),

               Commands.deadline(new PathsFollower("Depot Collect"),
                                 new FuelCollectorAutoCommand(robot,true, true,"in")),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),

                Commands.deadline(new PathsFollower("Depot Shoot"),
                                    new FuelCollectorAutoCommand(robot,true, false,"stop")),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),

                new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB),
                new PathsFollower("Depot Climb"),
                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),
                new PathsFollower("LC Adjust"),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),
                new ClimberAutoCommand(robot, true, 1.0, true)
        );


        Pose2d startPose = new PathsFollower("ALT-Depot").path.getStartingHolonomicPose().get();

        return new Autos.CommandAndPose(autoCommand, startPose);
    }
}