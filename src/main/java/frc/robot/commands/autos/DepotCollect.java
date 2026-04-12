package frc.robot.commands.autos;




import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;


public class DepotCollect {

    public static Autos.CommandAndPose depot_Collect(RobotContainer robot) {
        return depotRunCollect(robot, "H-Shoot", "H Depot Collect");
    }
    /*
    public static Autos.CommandAndPose depot_front(RobotContainer robot){
        return depotRunCollect(robot, "H-Depot", "Front Depot Collect");
    }

     */



    public static Autos.CommandAndPose depotRunCollect(
            RobotContainer robot,
            String path1,
            String path2) {

        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(


                plannerPath1,

                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                //new ShooterAutoCommand(robot).withTimeout(5),
                Commands.deadline(plannerPath2,
                        new FuelCollectorAutoCommand(robot, true, true, "in")),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                new ShooterAutoAimCommand(robot)
        );




        return new Autos.CommandAndPose(auto, startingPose);    }

}
