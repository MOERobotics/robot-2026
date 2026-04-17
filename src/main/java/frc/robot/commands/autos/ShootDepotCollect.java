package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autos;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.ShooterAutoAimCommand;
import frc.robot.container.RobotContainer;


public class ShootDepotCollect {
    public static Autos.CommandAndPose shoot(RobotContainer robot) {
        return shootAuto(robot, "H-Shoot", "H Depot Collect");
    }


    public static Autos.CommandAndPose shootAuto(
            RobotContainer robot,
            String path1,
            String path2){

        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(
                Commands.deadline(plannerPath1,
                        new FuelCollectorAutoCommand(robot,true,true, "stop")
                ),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),

                Commands.deadline(new ShooterAutoAimCommand(robot),
                     new FuelCollectorAutoCommand(robot, true, true, "in")
                ),
                plannerPath2,
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())


        );


        return new Autos.CommandAndPose(auto, startingPose);
    }

}
