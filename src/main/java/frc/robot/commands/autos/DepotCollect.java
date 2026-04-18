package frc.robot.commands.autos;




import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;

import static edu.wpi.first.units.Units.Meters;


public class DepotCollect {


/*
    public static Autos.CommandAndPose depot_front1(RobotContainer robot){
        return depotRunCollect(robot, "H-Depot", "Front Depot Collect", "Slow Front Collect");
    }
 */

    public static Autos.CommandAndPose depot_front2(RobotContainer robot){
        return depotRunCollect(robot, "H-Shoot", "H-Shot Collect Depo", "Slow Front Collect");
    }





    public static Autos.CommandAndPose depotRunCollect(
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
                Commands.run(() -> robot.getRobotSwerveDrive().stop()),
                Commands.deadline(new ShooterAutoAimCommand(robot),
                        new FuelCollectorAutoCommand(robot, true, true, "in")
                ),
                Commands.deadline(
                        Commands.sequence(
                             plannerPath2,
                                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                             new DistDriveCommand(robot, Meters.of(1),new ChassisSpeeds(1.5,0,0))
                             //   plannerPath2,
                                //  Commands.run(() -> robot.getRobotSwerveDrive().stop()).withTimeout(1),
                              //  plannerPath3
                        ),
                        new FuelCollectorAutoCommand(robot, true, true, "in")),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                new ShooterAutoAimCommand(robot)
                );




        return new Autos.CommandAndPose(auto, startingPose);    }

}
