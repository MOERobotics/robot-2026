package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.DepotRun;
import frc.robot.commands.autos.DepotRunTest;
import frc.robot.commands.autos.HubandBumpAutos;
import frc.robot.commands.autos.OutpostAutos;
import frc.robot.container.RobotContainer;

import java.util.function.Consumer;

public class Autos {
    public static SendableChooser<CommandAndPose> autoChooser = new SendableChooser<>();
    public static void setupAutos (RobotContainer robot){

        autoChooser.setDefaultOption("Auto1: H_RC ", HubandBumpAutos.H_RC(robot));
        autoChooser.setDefaultOption("Auto2: H_LC ", HubandBumpAutos.H_LC(robot));
        autoChooser.setDefaultOption("Auto3: RB_RC ", HubandBumpAutos.RB_RC(robot));
        autoChooser.setDefaultOption("Auto4: LB_LC ", HubandBumpAutos.LB_LC(robot));
        autoChooser.setDefaultOption("Auto5: DepotRun ", DepotRun.getAuto(robot));
        autoChooser.setDefaultOption("Auto6: DepotRunTest ", DepotRunTest.getAuto(robot));
        autoChooser.setDefaultOption("Auto7: Outpost ", OutpostAutos.outpost(robot));




        Consumer<CommandAndPose> onAutoUpdate = (commandandPose) -> robot.getRobotSwerveDrive().setPose(autoChooser.getSelected().pose());

        autoChooser.onChange(onAutoUpdate);
        onAutoUpdate.accept(autoChooser.getSelected());
        SmartDashboard.putData("Autos Chooser",autoChooser);

    }

    public static CommandAndPose getSelectedAuto () {
        return autoChooser.getSelected();
    }

    public record CommandAndPose(Command command, Pose2d pose){}
}
