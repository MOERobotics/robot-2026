package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ClimberAutoCommand;
import frc.robot.commands.FuelCollectorAutoCommand;
import frc.robot.commands.PathsFollower;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class AutosChooser {
    public RobotContainer robot;

    public static SendableChooser<Command> autoChooser = new SendableChooser<>();


    public static void setupAutos(RobotContainer robot) {

        autoChooser.setDefaultOption("TESTAuto1", Commands.sequence(
                Commands.deadline(
                        new PathsFollower("ALT-Depot"),
                        new FuelCollectorAutoCommand(robot,true, true,"in")
                ).withTimeout(3),

                Commands.runOnce(
                        () -> robot.getRobotSwerveDrive().stop()

                ),
                Commands.runOnce(
                        () -> System.err.println("\n\n\n\n\n\n\n\nDone!\n\n\n\n\n\n\n\n\n\n\n")
                )
                /*
                new PathsFollower("Depot Collect"),
                Commands.parallel(new PathsFollower("Depot Shoot"),
                        new FuelCollectorAutoCommand(robot,true, false,"stop")),

                new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB),

                new PathsFollower("Depot Climb"),
                new PathsFollower("LC Adjust"),
                new ClimberAutoCommand(robot, true, 1.0, true)

                 */


        ));


        Consumer<CommandAndPose> onAutoUpdate = (commandandPose) ->
                robot.getRobotSwerveDrive().
                        setPose(new PathsFollower("ALT-Depot").path.getStartingHolonomicPose().get());
//                        setPose(autoChooser.getSelected().
//                                path.getStartingHolonomicPose().get());
//
//        autoChooser.onChange(onAutoUpdate);
//        AutoUpdate.accept(autoChooser.getSelected());
        SmartDashboard.putData("Autos Chooser",autoChooser);


    }
/*

*/
    public Command getAuto() {
        return autoChooser.getSelected();
    }

    public record CommandAndPose(Command command, Pose2d pose) {
    }

}
