package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;

import static edu.wpi.first.units.Units.Seconds;


public class Outpost2 {

    public static Autos.CommandAndPose outpost(RobotContainer robot) {
        return buildOutpostAuto(robot, "ART-Outpost", "Outpost Collect", "Outpost Shoot", "Outpost Climb");
    }


    public static Autos.CommandAndPose buildOutpostAuto(
            RobotContainer robot,
            String path1,
            String path2,
            String path3,
            String path4) {
        // somehow this became the main auto for Seneca -> clean this up???

        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);
        PathsFollower plannerPath3 = new PathsFollower(path3);
        PathsFollower plannerPath4 = new PathsFollower(path4);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(
                Commands.parallel(
                        plannerPath1,
                        new FuelCollectorAutoCommand(robot, true, true, "stop").withTimeout(2)
                ),
                plannerPath2,
                Commands.deadline(
                        Commands.run(() -> robot.getRobotSwerveDrive().stop()).withTimeout(Seconds.of(5))
                        //,new FuelCollectorAutoCommand(robot, true, true, "in")
                        ),

                // new FuelCollectorAutoCommand(robot, true, true, "in").withTimeout(5),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                plannerPath3,
                Commands.deadline(
                        new ShooterAutoAimCommand(robot),
                    Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())
                ).withTimeout(Seconds.of(5)),
                plannerPath4,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())

                // new ClimberAutoCommand(robot, true, 1.0, true)

        );


        return new Autos.CommandAndPose(auto, startingPose);
    }

}
