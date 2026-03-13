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

import java.nio.file.Path;


public class Shoot {

    public static Autos.CommandAndPose shoot(RobotContainer robot) {
        return squareDanceAuto(robot, "H-LC");
    }


    public static Autos.CommandAndPose squareDanceAuto(
            RobotContainer robot,
            String path1){

        PathsFollower plannerPath1 = new PathsFollower(path1);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(
                //new FuelCollectorAutoCommand(robot, true, false, "stop").withTimeout(2),
                new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB)


        );


        return new Autos.CommandAndPose(auto, startingPose);
    }

}
