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


public class OutpostAutos {

    public static Autos.CommandAndPose outpost(RobotContainer robot) {
        return buildSnowBlowerAuto(robot, "ART-Outpost", "Outpost Collect", "Outpost Shoot", "Outpost Climb");
    }


    public static Autos.CommandAndPose buildSnowBlowerAuto(
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
                plannerPath1,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                Commands.deadline(
                        plannerPath2),
                        //new FuelCollectorAutoCommand(robot, true, true, "in")),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                plannerPath3,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                // new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB),
                plannerPath4,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())

                // new ClimberAutoCommand(robot, true, 1.0, true)

        );


        return new Autos.CommandAndPose(auto, startingPose);
    }

}
