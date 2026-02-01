// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.HubLoggingCommand;
import frc.robot.container.MiniBotContainer;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new MiniBotContainer();
    public Joystick driveJoystick = new Joystick(0);
    private CommandScheduler scheduler;

    public static final AprilTagFieldLayout kTagLayout;

    public Command hubLogging = new HubLoggingCommand();





    // imported field layout
    static {
        try {
            kTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getPath() + "/2026-rebuilt-welded.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }


    PhotonCamera photonCamera = new PhotonCamera("HD_Camera");

    List<PhotonPipelineResult> result;
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0,0,0), new Rotation3d());
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);


    VisionSystemSim visionSim;
    PhotonCameraSim cameraSim;
    SimCameraProperties cameraProps;


    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);

        MOELogger.setupLogging(this);


        scheduler = CommandScheduler.getInstance();
    }


    @Override
    public void driverStationConnected() {
    }

    @Override
    public void robotPeriodic() {
        MOELogger.log();
        scheduler.run();

        //reading all results from photon camera
        result = photonCamera.getAllUnreadResults();
        if (!result.isEmpty()) {
            // get latest results
            PhotonPipelineResult latestResult = result.get(result.size() - 1);

            Logger.recordOutput("HasTargets", latestResult.hasTargets());
            // creates list of poses seen by latest result of the camera
            List<Pose3d> targetPoses = new ArrayList<>();

            // finds estimated pose with multi target strategy
            Optional<EstimatedRobotPose> estimatedPose = photonEstimator.estimateCoprocMultiTagPose(latestResult);
            if (estimatedPose.isPresent()) {
                Pose3d pose = estimatedPose.get().estimatedPose;
                Logger.recordOutput("EstimatedPose", pose);
            }



            // makes list of targets and their data in advantage kit
            if (latestResult.hasTargets()) {

                // lists data for the best target
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

                Logger.recordOutput("Best Target Fiduciary ID", bestTarget.getFiducialId());
                Logger.recordOutput("Best Target Area", bestTarget.getArea());
                Logger.recordOutput("Best Target Yaw", bestTarget.getYaw());
                Logger.recordOutput("Best Target Pitch", bestTarget.getPitch());

                Logger.recordOutput("Best Camera to Target Translation",  bestTarget.getBestCameraToTarget());



                Pose3d bestRobotPose = new Pose3d();

                if (kTagLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
                    bestRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            bestTarget.getBestCameraToTarget(),
                            kTagLayout.getTagPose(bestTarget.getFiducialId()).get(),
                            kRobotToCam);
                }

                Logger.recordOutput("Best Target Pose ", bestRobotPose);

                // logs all target data
                for (PhotonTrackedTarget target : targets) {
                    int id = target.getFiducialId();
                    Logger.recordOutput("Targets/" + id + "/Area", target.getArea());
                    Logger.recordOutput("Targets/" + id + "/Yaw", target.getYaw());
                    Logger.recordOutput("Targets/" + id + "/Pitch", target.getPitch());
                    Logger.recordOutput("Targets/" + id + "/Skew", target.getSkew());

                    Pose3d robotPose = new Pose3d();

                    if (kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
                        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                target.getBestCameraToTarget(),
                                kTagLayout.getTagPose(target.getFiducialId()).get(),
                                kRobotToCam);
                    }
                    Logger.recordOutput("Targets/" + id + "/robotPose", robotPose);
                    targetPoses.add(robotPose);

                }



            }

        }


        scheduler.schedule(hubLogging);

    }



    @Override
    public void disabledInit() {
        scheduler.cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        double joystickX= driveJoystick.getX();
        double joystickY = -driveJoystick.getY();
        double leftPow = joystickX+joystickY;
        double rightPow = joystickY-joystickX;
        robot.getTankDrive().drive(leftPow,rightPow);

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        visionSim = new VisionSystemSim("main");


        visionSim.addAprilTags(
                kTagLayout
        );
        cameraProps = new SimCameraProperties();

        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(90));
        cameraProps.setFPS(20);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(photonCamera,cameraProps);
        visionSim.addCamera(cameraSim,kRobotToCam);
    }

    @Override
    public void simulationPeriodic() {
        Pose2d sim2Pose = robot.getTankDrive().getPose();
        visionSim.update(sim2Pose);
    }


}
