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

    public static Autos.CommandAndPose NLT_Back_Bump (RobotContainer robot) {
        return buildNeutral(robot, "NLT-Neutral", "LN Collect", "LN Return-1", "LN Return-2", "LA-Rotation", "L BackToN Bump");
    }

    public static Autos.CommandAndPose NLT_Back_Trench(RobotContainer robot) {
        return buildNeutral(robot, "NLT-Neutral", "LN Collect", "LN Return-1", "LN Return-2", "LA-Rotation", "L BackToN Trench");
    }

    public static Autos.CommandAndPose NRT_Back_Bump(RobotContainer robot) {
        return buildNeutral(robot, "NRT-Neutral", "RN Collect", "RN Return-1", "RN Return-2", "RA-Rotation", "R BackToN Bump");
    }
    public static Autos.CommandAndPose NRT_Back_Trench(RobotContainer robot) {
        return buildNeutral(robot, "ART-Neutral", "RN Collect", "RN Return-1","RN Return-2", "RA-Rotation", "R BackToN Trench");
    }
    public static Autos.CommandAndPose buildNeutral(
            RobotContainer robot,
            String path1,
            String path2,
            String path3,
            String path4,
            String path5,
            String path6) {

        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);
        PathsFollower plannerPath3 = new PathsFollower(path3);
        PathsFollower plannerPath4 = new PathsFollower(path4);
        PathsFollower plannerPath5 = new PathsFollower(path5);
        PathsFollower plannerPath6 = new PathsFollower(path6);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(

                new FuelCollectorAutoCommand(robot, true, true, "stop").withTimeout(1),
                Commands.deadline(
                        Commands.sequence(
                                plannerPath1,
                                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                                plannerPath2,
                                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())
                                ),
                        new FuelCollectorAutoCommand(robot ,true, true, "in")),
                plannerPath3,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                plannerPath4,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                plannerPath5,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                // TODO ADD SHOOT
                plannerPath6,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())
                //new ShooterAutoCommand(robot)
        );




        return new Autos.CommandAndPose(auto, startingPose);    }

}
