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

public class SnowBlowerAuto{

    public static Autos.CommandAndPose getAuto(RobotContainer robot) {

        Command autoCommand =  Commands.sequence(
                Commands.deadline(
                        new PathsFollower("ALT-Depot"),
                        new FuelCollectorAutoCommand(robot,true, true,"in")
                ).withTimeout(3),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),

                new PathsFollower("Depot Collect"),
                Commands.deadline(new PathsFollower("Depot Shoot"),
                        new FuelCollectorAutoCommand(robot,true, false,"stop")),

                new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB),
                new PathsFollower("Depot Climb"),
                new PathsFollower("LC Adjust"),
                new ClimberAutoCommand(robot, true, 1.0, true)



        );

        Pose2d startPose = new PathsFollower("ALT-Depot").path.getStartingHolonomicPose().get();

        return new Autos.CommandAndPose(autoCommand, startPose);
    }
}