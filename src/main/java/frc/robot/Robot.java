// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Converter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.container.MiniBotContainer;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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

    static {
        try {
            kTagLayout = new AprilTagFieldLayout("2026-rebuilt-welded.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }


    PhotonCamera frontCam = new PhotonCamera("PhotonCamera1");
    PhotonCamera rearCam = new PhotonCamera("PhotonCamera2");

    List<PhotonPipelineResult> result;
    public static final Transform3d kRobotToCamFront =
            new Transform3d(new Translation3d( Inches.of(0).in(Meter), Inches.of(0).in(Meter), Inches.of(7).in(Meter)), new Rotation3d(0,0, 0));
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCamFront);
    public static final Transform3d kRobotToCamRear =
            new Transform3d(new Translation3d( Inches.of(0).in(Meter), Inches.of(0).in(Meter), Inches.of(7).in(Meter)), new Rotation3d(0,0, Math.PI));

    VisionSystemSim visionSim;
    PhotonCameraSim frontCameraSim;
    PhotonCameraSim rearCameraSim;
    SimCameraProperties cameraProps;
    String matchState;
    List<Pose3d> frontTargetPoses;
    List<Pose3d> rearTargetPoses;



    @Override
    public void robotInit() {
        frontTargetPoses = new ArrayList<>();
        rearTargetPoses = new ArrayList<>();

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

        Logger.recordOutput("MatchDetails/Alliance", String.valueOf(DriverStation.getAlliance()));
        Logger.recordOutput("MatchDetails/CurrentMatchTime", DriverStation.getMatchTime());
        if (DriverStation.getMatchTime() >= 120) {
            matchState = "Auto";
        } else if (DriverStation.getMatchTime() < 120 && DriverStation.getMatchTime() >= 110) {
            matchState = "Transition";
        } else if (DriverStation.getMatchTime() < 110 && DriverStation.getMatchTime() >= 105) {
            matchState = "Shift 1";
        } else if (DriverStation.getMatchTime() < 105 && DriverStation.getMatchTime() >= 80) {
            matchState = "Shift 2";
        } else if (DriverStation.getMatchTime() < 80 && DriverStation.getMatchTime() >= 55) {
            matchState = "Shift 3";
        } else if (DriverStation.getMatchTime() < 55 && DriverStation.getMatchTime() >= 30) {
            matchState = "Shift 4";
        } else if (DriverStation.getMatchTime() < 30 && DriverStation.getMatchTime() >= 0) {
            matchState = "Endgame";
        } else {
            matchState = "Game Over";
        }
        Logger.recordOutput("MatchDetails/CurrentMatchState", matchState);
        ;

        MOELogger.log();
        scheduler.run();

        List<Optional<Pose3d>> cameraPoses = List.of(
                photonFunction(frontCam, kRobotToCamFront, kTagLayout, frontTargetPoses),
                photonFunction(rearCam, kRobotToCamRear, kTagLayout, rearTargetPoses)
        );
        List<Pose3d> validPoses = cameraPoses.stream()
                .flatMap(Optional::stream) // removes empty optionals
                .toList();
        Pose3d totalAveragePose = null;
        if (!validPoses.isEmpty()) {
            totalAveragePose = averagePoseMaker(validPoses);
        }


        Logger.recordOutput("TotalPose", totalAveragePose);


        /*
                Pose3d robotPose = new Pose3d();

                if (kTagLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            bestTarget.getBestCameraToTarget(),
                            kTagLayout.getTagPose(bestTarget.getFiducialId()).get(),
                            kRobotToCam);
                }
                Logger.recordOutput("Robot Pose", robotPose);
            }

         */


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

        frontCameraSim = new PhotonCameraSim(frontCam,cameraProps);
        rearCameraSim = new PhotonCameraSim(rearCam,cameraProps);




        visionSim.addCamera(frontCameraSim, kRobotToCamFront);
        visionSim.addCamera(rearCameraSim, kRobotToCamFront);
    }

    @Override
    public void simulationPeriodic() {
        Pose2d sim2Pose = robot.getTankDrive().getPose();
        visionSim.update(sim2Pose);
    }
    public Pose3d averagePoseMaker(List<Pose3d> listToAverage){
        double totalX = 0.0;
        double totalY = 0.0;
        double totalZ = 0.0;
        double totalSin = 0.0;
        double totalCos = 0.0;

        for (Pose3d pose : listToAverage) {
            totalX += pose.getX();
            totalY += pose.getY();
            totalZ += pose.getZ();

            double yaw = pose.getRotation().getZ();
            totalSin += Math.sin(yaw);
            totalCos += Math.cos(yaw);
        }


        Translation3d averageTranslation =
                new Translation3d(
                        totalX / listToAverage.size(),
                        totalY / listToAverage.size(),
                        totalZ / listToAverage.size());

        double avgYaw = Math.atan2(
                totalSin / listToAverage.size(),
                totalCos / listToAverage.size());

        Rotation3d averageRotation =
                new Rotation3d(0.0, 0.0, avgYaw);

        return new Pose3d(averageTranslation, averageRotation);

    }
    public Optional<Pose3d> photonFunction(PhotonCamera camera, Transform3d robotToCam, AprilTagFieldLayout tagLayout, List<Pose3d> targetsStoreList) {
        targetsStoreList.clear();
        result = camera.getAllUnreadResults();
        if (!result.isEmpty()) {
            PhotonPipelineResult latestResult = result.get(result.size() - 1);
            Logger.recordOutput(camera.getName() + "HasTargets", latestResult.hasTargets());

            if (latestResult.hasTargets()) {
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
                Logger.recordOutput(camera.getName() + "Best Target Fiduciary ID", bestTarget.getFiducialId());
                Logger.recordOutput(camera.getName() + "Best Target Area", bestTarget.getArea());
                Logger.recordOutput(camera.getName() + "Best Target Yaw", bestTarget.getYaw());
                Logger.recordOutput(camera.getName() + "Best Target Pitch", bestTarget.getPitch());

                Pose3d bestRobotPose = new Pose3d();

                if (tagLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
                    bestRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            bestTarget.getBestCameraToTarget(),
                            tagLayout.getTagPose(bestTarget.getFiducialId()).get(),
                            robotToCam);
                }

                Logger.recordOutput(camera.getName() + "Best Target Pose ", bestRobotPose);

                for (PhotonTrackedTarget target : targets) {
                    int id = target.getFiducialId();
                    Logger.recordOutput(camera.getName() + "Targets/" + id + "/Area", target.getArea());
                    Logger.recordOutput(camera.getName() + "Targets/" + id + "/Yaw", target.getYaw());
                    Logger.recordOutput(camera.getName() + "Targets/" + id + "/Pitch", target.getPitch());
                    Logger.recordOutput(camera.getName() + "Targets/" + id + "/Skew", target.getSkew());

                    Pose3d robotPose = new Pose3d();

                    if (tagLayout.getTagPose(target.getFiducialId()).isPresent()) {
                        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                target.getBestCameraToTarget(),
                                tagLayout.getTagPose(target.getFiducialId()).get(),
                                robotToCam);
                    }
                    Logger.recordOutput(camera.getName() + "Targets/" + id + "/robotPose", robotPose);
                    targetsStoreList.add(robotPose);
                }
                Pose3d averagePose = averagePoseMaker(targetsStoreList);

                Logger.recordOutput(camera.getName() + "AveragePose", averagePose);
                targetsStoreList.clear();
                return Optional.of(averagePose);
            }
        }
        return Optional.empty();
    }
}
