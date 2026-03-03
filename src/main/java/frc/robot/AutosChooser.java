package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PathsFollower;
import frc.robot.container.RobotContainer;

import java.util.function.Consumer;

public class AutosChooser {
    public RobotContainer robot;

    public static SendableChooser<PathsFollower> autoChooser = new SendableChooser<>();

    public static void setupAutos(RobotContainer robot) {

        autoChooser.setDefaultOption("Auto1", new PathsFollower("Curved Path"));
        autoChooser.addOption("Auto2", new PathsFollower("Example Path"));
        autoChooser.addOption("Auto3", new PathsFollower("Test Path"));
        autoChooser.addOption("Auto4", new PathsFollower("New Path"));
        Consumer<CommandAndPose> onAutoUpdate = (commandandPose) ->
                robot.getRobotSwerveDrive().
                        setPose(autoChooser.getSelected().
                                path.getStartingHolonomicPose().get());
        //autoChooser.onChange(onAutoUpdate);
        //nAutoUpdate.accept(autoChooser.getSelected());
        SmartDashboard.putData("Autos Chooser",autoChooser);
    }
/*

*/
    public PathsFollower getAuto() {
        return autoChooser.getSelected();
    }

    public record CommandAndPose(Command command, Pose2d pose) {
    }

}
