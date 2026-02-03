package frc.robot.subsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import frc.robot.MOESubsystem;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// Need to construct the Photon Cam as a class and extend Autologging and PhotonCameraSubsystem
public class PhotonCameraObject extends MOESubsystem<VisionInputsAutoLogged> implements PhotonCameraSubsystem {


    public VisionSystemSim visionSim;
    public PhotonCameraSim cameraSim;
    public SimCameraProperties cameraProps;
    public  PhotonCamera camera;
    public static final AprilTagFieldLayout kTagLayout;
    static {
        try {
            kTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getPath() + "/2026-rebuilt-welded.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    List<PhotonPipelineResult> result;

    public  Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0,0,0), new Rotation3d());
    public PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);


    public PhotonCameraObject(String cameraName) {
        super(new VisionInputsAutoLogged());
        camera = new PhotonCamera(cameraName);


    }


    @Override
    public boolean hasTargets() {
        return getSensors().hasTargets;
    }

    @Override
    public void periodic() {
        result = camera.getAllUnreadResults();
        if (!result.isEmpty()) {
            // get latest results
            PhotonPipelineResult latestResult = result.get(result.size() - 1);

            Logger.recordOutput("HasTargets", latestResult.hasTargets());
            // creates list of poses seen by latest result of the cameraAmbiguity
            List<Pose3d> targetPoses = new ArrayList<>();

            // finds estimated pose with multi target strategy
            Pose3d pose = new Pose3d();
            Optional<EstimatedRobotPose> estimateMultiPose = photonEstimator.estimateCoprocMultiTagPose(latestResult);
            Optional<EstimatedRobotPose> estimateAmbPose = photonEstimator.estimateLowestAmbiguityPose(latestResult);

            if (estimateMultiPose.isPresent()) {
                pose = estimateMultiPose.get().estimatedPose;
                getSensors().hasMultiEstimatedPose = true;
            } else if(estimateAmbPose.isPresent()) {
                getSensors().hasMultiEstimatedPose = false;
                pose = estimateAmbPose.get().estimatedPose;
            } else{
                pose = Pose3d.kZero;
            }

            Logger.recordOutput("EstimatedPose", pose);
            getSensors().estimatedPose = pose;


            // makes list of targets and their data in advantage kit
            if (latestResult.hasTargets()) {

                // lists data for the best target
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

                Logger.recordOutput("Best Target Fiduciary ID", bestTarget.getFiducialId());
                Logger.recordOutput("Best Target Area", bestTarget.getArea());
                Logger.recordOutput("Best Target Yaw", bestTarget.getYaw());
                Logger.recordOutput("Best Target Pitch", bestTarget.getPitch());

                Logger.recordOutput("Best Camera to Target Transform", bestTarget.getBestCameraToTarget());

                getSensors().bestArea = bestTarget.getArea();
                getSensors().bestPitch = bestTarget.getPitch();
                getSensors().bestYaw = bestTarget.getYaw();
                getSensors().bestFiducialId = bestTarget.getFiducialId();


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
    }




    @Override
    public Optional<Pose3d> getEstimatedPose() {

        if (getSensors().hasMultiEstimatedPose) {
            return Optional.of(getSensors().estimatedPose);
        } else {
            return Optional.empty();
        }

    }



}
