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


public class NeutralZone {

    public static Autos.CommandAndPose NLT(RobotContainer robot) {
        return buildNeutral(robot, "NLT-Neutral", "LN Collect", "RN Return");
    }

    public static Autos.CommandAndPose NRT(RobotContainer robot) {
        return buildNeutral(robot, "NRT-Neutral", "RN Collect", "LN Return");
    }

    public static Autos.CommandAndPose NRT_Back(RobotContainer robot) {
        return buildNeutral(robot, "NRT-Neutral", "RN Collect", "RN Return");
    }
    public static Autos.CommandAndPose buildNeutral(
            RobotContainer robot,
            String path1,
            String path2,
            String path3) {

        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);
        PathsFollower plannerPath3 = new PathsFollower(path3);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(


                plannerPath1,

                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                Commands.deadline(
                        plannerPath2,
                        new FuelCollectorAutoCommand(robot ,true, true, "in")),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                plannerPath3,

                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())



        );




        return new Autos.CommandAndPose(auto, startingPose);    }

}
