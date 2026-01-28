// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new MiniBotContainer();
    public Joystick driveJoystick = new Joystick(0);
    private CommandScheduler scheduler;

    public static final AprilTagFieldLayout kTagLayout;


    DriverStation driverStation;

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
            new Transform3d(new Translation3d(Inches.of(7).in(Meter), Inches.of(2).in(Meter), Inches.of(7).in(Meter)), new Rotation3d(15, 0, 0));
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


        result = photonCamera.getAllUnreadResults();
        if (!result.isEmpty()) {
            PhotonPipelineResult latestResult = result.get(result.size() - 1);
            Logger.recordOutput("HasTargets", latestResult.hasTargets());

            List<Pose3d> targetPoses = new ArrayList<>();
            Optional<EstimatedRobotPose> estimatedPose = photonEstimator.estimateCoprocMultiTagPose(latestResult);

            if (estimatedPose.isPresent()) {
                Pose3d pose = estimatedPose.get().estimatedPose;
                Logger.recordOutput("Vision/EstimatedPose", pose);
            }


            if (latestResult.hasTargets()) {
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
                Logger.recordOutput("Best Target Fiduciary ID", bestTarget.getFiducialId());
                Logger.recordOutput("Best Target Area", bestTarget.getArea());
                Logger.recordOutput("Best Target Yaw", bestTarget.getYaw());
                Logger.recordOutput("Best Target Pitch", bestTarget.getPitch());

                Pose3d bestRobotPose = new Pose3d();

                if (kTagLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
                    bestRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            bestTarget.getBestCameraToTarget(),
                            kTagLayout.getTagPose(bestTarget.getFiducialId()).get(),
                            kRobotToCam);
                }

                Logger.recordOutput("Best Target Pose ", bestRobotPose);

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

        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Time", matchTime);

        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        String allianceStr = " ";

        if (allianceOpt.isPresent()) {
            if (allianceOpt.get() == DriverStation.Alliance.Red) {
                allianceStr = "Red";
            } else if (allianceOpt.get() == DriverStation.Alliance.Blue) {
                allianceStr = "Blue";
            }
        }

        SmartDashboard.putString("Match Alliance", allianceStr);


        String gameData = DriverStation.getGameSpecificMessage();
        char autoLoser =  ' ';
        if(!gameData.isEmpty()){
            autoLoser = gameData.charAt(0);
        }

        if (autoLoser == 'R') {
            SmartDashboard.putString("First Inactive", "Red");
        } else if (autoLoser == 'B')
            SmartDashboard.putString("First Inactive", "Blue");

        else {
            SmartDashboard.putString("First Inactive", "None");
        }

        int shift = 0;

        if (matchTime <= 135 && matchTime > 110) {
            shift = 1;

        } else if(matchTime <= 110 && matchTime > 85){
            shift = 2;

    } else if(matchTime <=85&&matchTime >60){
            shift =3;
    }else if (matchTime <= 60 && matchTime > 35) {
            shift = 4;
        }

        SmartDashboard.putNumber("Current Shift", shift);


        boolean redHubActive = true;
        boolean blueHubActive = true;

        if (shift != 0 && (autoLoser == 'R' || autoLoser == 'B')) {

            boolean redActiveThisShift;

            if (shift == 2 || shift == 4){
                redActiveThisShift=true;
            }else{
                redActiveThisShift=false;
            }


            if (autoLoser == 'R') {
                redHubActive = redActiveThisShift;
                blueHubActive = !redActiveThisShift;
            } else {
                blueHubActive = redActiveThisShift;
                redHubActive = !redActiveThisShift;
            }

        }

        SmartDashboard.putBoolean("Red Hub Active", redHubActive);
        SmartDashboard.putBoolean("Blue Hub Active", blueHubActive);

        boolean ourHubActive = false;

        if (allianceOpt.isPresent()) {
            if (allianceOpt.get() == DriverStation.Alliance.Red && redHubActive) {

                ourHubActive = true;

            } else if (allianceOpt.get() == DriverStation.Alliance.Blue && blueHubActive) {

                ourHubActive = true;
            }
        }

        SmartDashboard.putBoolean("Our Hub Active", ourHubActive);



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
