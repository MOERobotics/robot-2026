// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.container.MiniBotContainer;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

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

    PhotonCamera photonCamera = new PhotonCamera("PhotonCamera1");

    List<PhotonPipelineResult> result;
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

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

        result = photonCamera.getAllUnreadResults();
        if (!result.isEmpty()) {
            PhotonPipelineResult latestResult = result.get(result.size() - 1);
            Logger.recordOutput("Has Targets", latestResult.hasTargets());
            if (latestResult.hasTargets()) {
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
                Logger.recordOutput("Target Area", targets.get(0).getArea());
                Logger.recordOutput("Best Target Area", bestTarget.getArea());
                Logger.recordOutput("Best Target Yaw", bestTarget.getYaw());
                Logger.recordOutput("Best Target Pitch", bestTarget.getPitch());
                Logger.recordOutput("Best Target Fiducial Id", bestTarget.getFiducialId());


                Pose3d robotPose = new Pose3d();

                if (kTagLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            bestTarget.getBestCameraToTarget(),
                            kTagLayout.getTagPose(bestTarget.getFiducialId()).get(),
                            kRobotToCam);
                }
                Logger.recordOutput("Robot Pose", robotPose);
            }

        /*
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : photonCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }


         */
            // boolean connected = photonCamera.isConnected();
            // Logger.recordOutput("Camera Connected", connected);
        }

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
    }

    @Override
    public void simulationPeriodic() {
    }


}
