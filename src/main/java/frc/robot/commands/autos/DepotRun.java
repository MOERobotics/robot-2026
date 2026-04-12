package frc.robot.commands.autos;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;

public class DepotRun{

    public static Autos.CommandAndPose getAuto(RobotContainer robot) {

        PathsFollower ALT_DEPOT = new PathsFollower("ALT-Depot");

        Pose2d startingPoseBlue = ALT_DEPOT.path.getStartingHolonomicPose().get();




        Command autoCommand =  Commands.sequence(
                Commands.deadline(
                        ALT_DEPOT,
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

                new ShooterTeleopAutoAimCommand(robot),
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



        return new Autos.CommandAndPose(autoCommand, startingPoseBlue);
    }
}