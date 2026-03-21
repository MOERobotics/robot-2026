// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.container.ProMOEtheus;
import frc.robot.commands.autos.DepotRun;
import frc.robot.commands.autos.DepotRunTest;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/*
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 */

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.RPM;




public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;
    private Command collectorTestCommand = new FuelCollectorTestCommand(robot, functionJoystick);
    private Command collectorTeleopCommand = new FuelCollectorTeleopCommand(robot, functionJoystick);

    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);

    private Command shooterTestCommand = new ShooterTestCommand(robot, driverJoystick, functionJoystick);

    private Command shooterTeleopCommand = new ShooterTeleopCommand(robot, functionJoystick);

    public Command rotateCommand = new AutoRotateCommand(robot, driverJoystick);

    public Command driveTeleopCommand = new DriveTeleopCommand(robot, driverJoystick);


    public Command hubLoggingCommand = new HubLoggingCommand(driverJoystick);
    public Command autoRotate = new AutoRotateCommand(robot, driverJoystick);


    public Command autoShootercommand = new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB, false);

    public PathsFollower testPath = new PathsFollower("Curved Path");

    public Command climberTeleopCommand = new ClimberTeleopCommand(robot, driverJoystick);

    public Command climberAutoCommand = new ClimberAutoCommand(robot, true, 1.0, false);

    public Command collectorAutoCommand = new FuelCollectorAutoCommand(robot, true, false, "out");

    //AutosChooser autoCommand = new AutosChooser();
    private Autos.CommandAndPose auto;

    Autos.CommandAndPose autoCommand = new Autos.CommandAndPose(Commands.none(), new Pose2d());

    PhotonCamera _camera1 = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");

    PhotonCamera _camera2 = new PhotonCamera("Arducam_OV9281_USB_Camera");

    CameraInputsAutoLogged cameraInputsAutoLogged = new CameraInputsAutoLogged();
   // public AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout();

  //  public Transform3d robotToCam = new Transform3d(0,0,0 ,new Rotation3d(0,0,0));
   //  public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,robotToCam );

    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
        MOELogger.setupLogging(this);
        scheduler = CommandScheduler.getInstance();
//        scheduler.schedule(FollowPathCommand.warmupCommand());
        scheduler.schedule(hubLoggingCommand);

        // auto = DepotRunTest.getAuto(robot);

        //  auto = OutpostAutos.outpost(robot);

    }


    @Override
    public void driverStationConnected() {
        Autos.setupAutos(robot);

    }

    long _heartbeat = 0;
    @Override
    public void robotPeriodic() {
        MOELogger.log();
        scheduler.run();

        if (autoCommand != null) {
            Logger.recordOutput("command", autoCommand.command().getName());

        }

        /*
       List<PhotonPipelineResult> results = _camera.getAllUnreadResults();

        if(!results.isEmpty()){
           PhotonPipelineResult latest =  results.get(results.size()-1);

           if(latest.hasTargets()) {
               PhotonTrackedTarget bestTarget = latest.getBestTarget();
               SmartDashboard.putBoolean("Vision/hasTargets", latest.hasTargets());
               SmartDashboard.putNumber("Vision/BestTargetID", bestTarget.getFiducialId());
               SmartDashboard.putNumber("Vision/Pitch", bestTarget.pitch);
               SmartDashboard.putNumber("Vision/Yaw", bestTarget.yaw);
               SmartDashboard.putNumber("Vision/skew", bestTarget.skew);
               Pose3d averageBestTargetPose = photonPoseEstimator.estimateAverageBestTargetsPose(latest).get().estimatedPose;
                Pose3d tagPose = fieldLayout.getTagPose(bestTarget.fiducialId).get();
               Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), tagPose, robotToCam);
               SmartDashboard.putData("Vision/averageBestTargetPose", (Sendable) averageBestTargetPose);
               SmartDashboard.putData("Vision/RobotPose", (Sendable) robotPose);

           }

         */



        List<PhotonPipelineResult> results1 = _camera1.getAllUnreadResults();
        if (!results1.isEmpty()) {
            Logger.recordOutput(
                    "photon1",
                    PhotonPipelineResult.proto,
                    results1.get(results1.size()-1)
            );
            cameraInputsAutoLogged.photon1 = results1.get(results1.size()-1);
        }
        List<PhotonPipelineResult> results2 = _camera2.getAllUnreadResults();

        if (!results2.isEmpty()) {
            Logger.recordOutput(
                    "photon2",
                    PhotonPipelineResult.proto,
                    results2.get(results2.size()-1)
            );
            cameraInputsAutoLogged.photon2 = results2.get(results2.size()-1);
        }
        Logger.processInputs("photonStuff", cameraInputsAutoLogged);

    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
    }

    @Override
    public void disabledPeriodic() {
        setFieldPose();
    }

    @Override
    public void autonomousInit() {
        //      scheduler.schedule(autoShootercommand);

        // scheduler.schedule(collectorAutoCommand);
        //scheduler.schedule(climberAutoCommand);
        // testPath =  new PathsFollower("ALT-Depot");
        // robot.getRobotSwerveDrive().setPose( testPath.path.getStartingHolonomicPose().get());
        //scheduler.schedule(autoCommand.getAuto());
/*
        robot.getTankDrive().setPose(testPath.path.getStartingDifferentialPose());
        scheduler.schedule(testPath);
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingDifferentialPose());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());

 */


        autoCommand = Autos.getSelectedAuto();


        setFieldPose();

        scheduler.schedule(autoCommand.command());




/*


        robot.getRobotSwerveDrive().setPose(auto.pose());
        scheduler.schedule(auto.command());


 */


    }

    @Override
    public void autonomousPeriodic() {

        /*
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingHolonomicPose().get());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());
        Logger.recordOutput("idfk lmao", testPath.path.getPathPoses().toArray(Pose2d[]::new));

         */

    }

    @Override
    public void teleopInit() {

        scheduler.schedule(climberTestCommand);

        scheduler.schedule(shooterTeleopCommand);
        scheduler.schedule(collectorTeleopCommand);
       // scheduler.schedule(climberTeleopCommand);

    }

    @Override
    public void teleopPeriodic() {
/*
        ChassisSpeeds robotSpeed = new ChassisSpeeds(
       /* ChassisSpeeds robotSpeed = new ChassisSpeeds(
                MathUtil.applyDeadband(driverJoystick.getRawAxis(1) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(0) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(2) * -1, deadband));

        robot.getRobotSwerveDrive().robotDrive(robotSpeed, false);


 */

        if (driverJoystick.getRawButton(1)) {
            robot.getRobotSwerveDrive().setPose(new Pose2d(robot.getRobotSwerveDrive().getPose().getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? Rotation2d.kZero : Rotation2d.kPi));
        }

        if (driverJoystick.getPOV() != -1) {
            scheduler.cancel(driveTeleopCommand);
            scheduler.schedule(rotateCommand);

        } else {
            {
                scheduler.cancel(autoRotate);
                scheduler.schedule(driveTeleopCommand);
            }
        }


    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        scheduler.schedule(climberTestCommand);
        scheduler.schedule(shooterTestCommand);
        scheduler.schedule(collectorTestCommand);
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }


    public void setFieldPose() {
        assert autoCommand != null;
        Pose2d startingPoseBlue = autoCommand.pose();
        final Pose2d startingPose;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            startingPose = FlippingUtil.flipFieldPose(startingPoseBlue);
        } else {
            startingPose = startingPoseBlue;
        }
        robot.getRobotSwerveDrive().setPose(startingPose);


    }




}