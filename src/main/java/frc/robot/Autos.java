package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.*;
import frc.robot.container.RobotContainer;

import java.util.function.Consumer;

public class Autos {
    public static SendableChooser<CommandAndPose> autoChooser = new SendableChooser<>();
    public static void setupAutos (RobotContainer robot){

   //     autoChooser.setDefaultOption("Auto1: H_RC ", HubandBumpAutos.H_RC(robot));
     //   autoChooser.addOption("Auto2: H_LC ", HubandBumpAutos.H_LC(robot));
      //  autoChooser.addOption("Auto3: RB_RC ", HubandBumpAutos.RB_RC(robot));
    //    autoChooser.addOption("Auto4: LB_LC ", HubandBumpAutos.LB_LC(robot));
        autoChooser.addOption("Auto5: DepotRun ", DepotRun.getAuto(robot));
      //  autoChooser.addOption("Auto6: DepotRunTest ", DepotRunTest.getAuto(robot));
      //  autoChooser.addOption("Auto7: Outpost ", OutpostAutos.outpost(robot));
      //  autoChooser.addOption("Auto8: SquareDance", SquareDance.squareDance(robot));

        autoChooser.addOption("Auto9: Justshoot", Shoot.shoot(robot));
        autoChooser.addOption("Auto10: Outpostw/Shooting", Outpost2.outpost(robot));

        autoChooser.addOption("Auto11: Neutral Zone NLT Bump", NeutralZone.NLT_Back_Bump(robot));

        autoChooser.addOption("Auto12: Neutral Zone NLT Trench", NeutralZone.NLT_Back_Trench(robot));

        autoChooser.addOption("Auto13: Neutral Zone NRT Bump", NeutralZone.NRT_Back_Bump(robot));

        autoChooser.addOption("Auto14: Neutral Zone NRT Trench", NeutralZone.NRT_Back_Trench(robot));


        autoChooser.addOption("Auto13: Neutral Zone ART Bump", NeutralZone.ART_Back_Bump(robot));

        autoChooser.addOption("Auto14: Neutral Zone ART Trench", NeutralZone.ART_Back_Trench(robot));


     //   autoChooser.addOption("Auto15: Drive 15ft", DriveForward.drive15Ft(robot));

     //   autoChooser.addOption("Auto16: FlywheelRates", FlywheelRates.percent10inc(robot));

        autoChooser.addOption("Auto17: DepotCollect", DepotCollect.depot_front(robot));

        autoChooser.addOption("Auto18: NeutralAuto Right", NeutralZone.NRT_Trench_Back_Trench(robot));

        autoChooser.addOption("Auto19: NeutralAuto Left", NeutralZone.NLT_Trench_Back_Trench(robot));


        //  autoChooser.setDefaultOption("Auto15: Drive 15ft", DriveForward.drive15Ft(robot));



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
