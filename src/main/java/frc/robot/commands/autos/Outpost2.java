package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.ClimberAutoCommand;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.container.RobotContainer;


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


        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);
        PathsFollower plannerPath3 = new PathsFollower(path3);
        PathsFollower plannerPath4 = new PathsFollower(path4);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(
                Commands.parallel(
                        plannerPath1,
                        new FuelCollectorAutoCommand(robot, true, true, "stop").withTimeout(5)),

                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                        plannerPath2,

                        //,new FuelCollectorAutoCommand(robot, true, true, "in").withTimeout(5)


                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),

                plannerPath3,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),

                new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB),
                plannerPath4,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())

                // new ClimberAutoCommand(robot, true, 1.0, true)

        );


        return new Autos.CommandAndPose(auto, startingPose);
    }

}
