package frc.robot.subsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CameraControl extends MOESubsystem<VisionInputsAutoLogged> implements PhotonCameraSubsystem {

    //TODO change name

    public enum VisionPoseSource {
        NONE,
        MULTI_TAG,
        SINGLE_TAG
    }
    public final PhotonCamera camera;
    public final PhotonPoseEstimator photonEstimator;
    public final Transform3d robotToCam;

    public static final AprilTagFieldLayout kTagLayout;

    static {
        try {
            kTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getPath()  + "/2026-rebuilt-welded.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public CameraControl(Transform3d robotToCam, String cameraName) {

        super(new VisionInputsAutoLogged());

        this.robotToCam = robotToCam;
        this.camera = new PhotonCamera(cameraName);
        this.photonEstimator = new PhotonPoseEstimator(kTagLayout, robotToCam);
    }

    @Override
    public void periodic() {

        List<PhotonPipelineResult> results =
                camera.getAllUnreadResults();

        if (results.isEmpty()) return;

        PhotonPipelineResult latest =
                results.get(results.size() - 1);

        getSensors().hasTargets = latest.hasTargets();
        Logger.recordOutput("Vision/HasTargets", latest.hasTargets());

        updateMultiTagPose(latest);

        if (latest.hasTargets()) {
            updateSingleTagPose(latest);
            processAllTargets(latest);
        } else {
            getSensors().hasSingleTagPose = false;
        }
        selectFinalVisionPose();
        logVisionSummary();
    }


    private void updateMultiTagPose(PhotonPipelineResult result) {

        Optional<EstimatedRobotPose> multi = photonEstimator.estimateCoprocMultiTagPose(result);

        if (multi.isPresent()) {

            getSensors().multiTagPose = multi.get().estimatedPose;

            getSensors().hasMultiTagPose = true;

        } else {
            multi = photonEstimator.estimateLowestAmbiguityPose(result);
            getSensors().hasMultiTagPose = false;
        }

        Logger.recordOutput("Vision/MultiTagPose", getSensors().multiTagPose);
    }


    private void updateSingleTagPose(PhotonPipelineResult result) {

        PhotonTrackedTarget best = result.getBestTarget();

        getSensors().bestArea = best.getArea();
        getSensors().bestPitch = best.getPitch();
        getSensors().bestYaw = best.getYaw();
        getSensors().bestFiducialId =
                best.getFiducialId();

        Pose3d pose =
                estimateRobotPoseFromTarget(best);

        getSensors().singleTagPose = pose;
        getSensors().hasSingleTagPose = true;

        Logger.recordOutput("Vision/SingleTagPose", pose);
    }

    private void selectFinalVisionPose() {

        if (getSensors().hasMultiTagPose) {

            getSensors().finalVisionPose = getSensors().multiTagPose;

            getSensors().finalPoseSource = VisionPoseSource.MULTI_TAG.name();

        } else if (getSensors().hasSingleTagPose) {

            getSensors().finalVisionPose =
                    getSensors().singleTagPose;

            getSensors().finalPoseSource = VisionPoseSource.SINGLE_TAG.name();

        } else {

            getSensors().finalVisionPose = Pose3d.kZero;

            getSensors().finalPoseSource = VisionPoseSource.NONE.name();
        }
    }


    private void logVisionSummary() {

        Logger.recordOutput("Vision/FinalPose", getSensors().finalVisionPose);

        Logger.recordOutput("Vision/FinalPoseSource", getSensors().finalPoseSource);
    }


    private void processAllTargets(
            PhotonPipelineResult result) {

        List<Pose3d> targetPoses = new ArrayList<>();

        for (PhotonTrackedTarget target :
                result.getTargets()) {

            int id = target.getFiducialId();

            Logger.recordOutput("Vision/Targets/" + id + "/Area", target.getArea());

            Logger.recordOutput("Vision/Targets/" + id + "/Yaw", target.getYaw());

            Logger.recordOutput("Vision/Targets/" + id + "/Pitch", target.getPitch());

            Pose3d pose = estimateRobotPoseFromTarget(target);

            Logger.recordOutput("Vision/Targets/" + id + "/RobotPose", pose);

            targetPoses.add(pose);
        }
    }


    private Pose3d estimateRobotPoseFromTarget(
            PhotonTrackedTarget target) {

        Optional<Pose3d> tagPose = kTagLayout.getTagPose(target.getFiducialId());

        if (tagPose.isPresent()) {

            return PhotonUtils.estimateFieldToRobotAprilTag(
                     target.getBestCameraToTarget(),
                            tagPose.get(),
                            robotToCam);
        }

        return Pose3d.kZero;
    }


    public Optional<Pose3d> getFinalVisionPose() {

        if (getSensors().finalPoseSource.equals(VisionPoseSource.NONE.name()))
            return Optional.empty();

        return Optional.of(getSensors().finalVisionPose);
    }

    public Optional <Double> angleToTarget(){
        Logger.recordOutput("Angle", PhotonUtils.getYawToPose(this.getFinalVisionPose().get().toPose2d(), kTagLayout.getTagPose(1).get().toPose2d()).getDegrees());

        if(getFinalVisionPose().isPresent()){
            return Optional.of(PhotonUtils.getYawToPose(getSensors().singleTagPose.toPose2d(), kTagLayout.getTagPose(1).get().toPose2d()).getDegrees());

        }else {
            return Optional.empty();
        }

    }
}
