package frc.robot.commands.autos;


import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autos;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.ShooterAutoAimCommand;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;


public class NeutralZoneSinglePath {

    public static Autos.CommandAndPose NeutralMegaPathRight (RobotContainer robot) {
        return buildNeutral(robot, "Right Neutral 1");
    }
    public static Autos.CommandAndPose NeutralMegaPathLeft (RobotContainer robot) {
        return buildNeutral(robot, "Left Neutral 1");
    }

    public static Autos.CommandAndPose buildNeutral(
            RobotContainer robot,
            String path1) {

        HashMap<String, Command> triggerMap = new HashMap<>();

        PathsFollower plannerPath1 = new PathsFollower(path1);

        Pose2d startingPose = plannerPath1.path.getStartingHolonomicPose().get();

        //triggerMap.put("Run Rollers", new FuelCollectorAutoCommand(robot, true, true, "in"))
        /*
        new EventTrigger("Run Roller").whileTrue(
                new FuelCollectorAutoCommand(
                        robot,
                        true,
                        true,
                        "in").alongWith(Commands.runOnce(() -> Logger.recordOutput("triggered", true)))).onFalse(
                new FuelCollectorAutoCommand(
                        robot,
                        true,
                        true,
                        "stop"));

         */


        Command auto = Commands.sequence(


                Commands.runOnce(() -> Logger.recordOutput("Auto Target Poses", plannerPath1.path.getPathPoses().toArray(Pose2d[]::new))),
                new FuelCollectorAutoCommand(robot, false, true, "stop").withTimeout(0.5),
                Commands.deadline(
                plannerPath1,
                        new FuelCollectorAutoCommand(robot, true, true, "in")),
                //new FuelCollectorAutoCommand(robot ,true, true, "in")),
                Commands.runOnce(() -> robot.getRobotSwerveDrive().stop()),
                // TODO ADD SHOOT
                new ShooterAutoAimCommand(robot)
        );




        return new Autos.CommandAndPose(auto.withName("Neutral"+path1), startingPose);    }

}
