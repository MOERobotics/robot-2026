package frc.robot.commands.autos;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Autos;
import frc.robot.commands.ClimberAutoCommand;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.container.RobotContainer;


public class HubandBumpAutos {

    public static Autos.CommandAndPose H_RC(RobotContainer robot) {
        return buildHubandBumpAutos(robot, "H-RC", "RC Adjust");
    }
    public static Autos.CommandAndPose H_LC(RobotContainer robot) {
        return buildHubandBumpAutos(robot, "H-LC", "LC Adjust");
    }
    public static Autos.CommandAndPose RB_RC(RobotContainer robot) {
        return buildHubandBumpAutos(robot, "RB-RC", "RC Adjust");
    }
    public static Autos.CommandAndPose LB_LC(RobotContainer robot) {
        return buildHubandBumpAutos(robot, "LB-LC", "LC Adjust");
    }



    public static Autos.CommandAndPose buildHubandBumpAutos(
            RobotContainer robot,
            String path1,
            String path2) {

        PathsFollower plannerPath1 = new PathsFollower(path1);
        PathsFollower plannerPath2 = new PathsFollower(path2);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();


        Command auto = Commands.sequence(
               // new ShooterAutoCommand(robot).withTimeout(5),


                plannerPath1,

                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop())

                /*

                ,
                plannerPath2,

                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),

                new ClimberAutoCommand(robot, true, 1.0, true)

                 */

        );




        return new Autos.CommandAndPose(auto, startingPose);    }

}
