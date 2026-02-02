package frc.robot.subsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

// Need to construct the Photon Cam as a class and extend Autologging and PhotonCameraSubsystem
public class Vision extends MOESubsystem<CameraInputsAutoLogged> implements VisionSubsystem{
    public PhotonCamera camera;
    public Transform3d camToRobot;
    public AprilTagFieldLayout tagFieldLayout;
    public List<Pose3d> targetPoses;
    public VisionSystemSim visionSim;

    public Vision () throws IOException {
        super(new CameraInputsAutoLogged());
        this.camera  = new PhotonCamera("HDCamera");
        this.camToRobot = new Transform3d(
                new Translation3d(
                        Inches.of(0).in(Meter),
                        Inches.of(0).in(Meter),
                        Inches.of(7).in(Meter)),
                new Rotation3d(0,0, 0));
        this.tagFieldLayout  = new AprilTagFieldLayout(Filesystem.getDeployDirectory() +
                "/2026-rebuilt-welded.json");
        /*
        static {
            try {
                tagFieldLayout  = new AprilTagFieldLayout(
                        Filesystem.getDeployDirectory() +
                                "/2026-rebuilt-welded.json");
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
*/
        this.targetPoses = new ArrayList<>();
        this.visionSim = new VisionSystemSim("main");
    }

    @Override
    public Optional<Pose3d> photonFunction() {
        targetPoses.clear();
        var result = camera.getAllUnreadResults();
        if (!result.isEmpty()) {
            PhotonPipelineResult latestResult = result.get(result.size() - 1);
            Logger.recordOutput(camera.getName() + "HasTargets", latestResult.hasTargets());

            if (latestResult.hasTargets()) {
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
                Logger.recordOutput(camera.getName() + "/Best Target Fiduciary ID", bestTarget.getFiducialId());
                Logger.recordOutput(camera.getName() + "/Best Target Area", bestTarget.getArea());
                Logger.recordOutput(camera.getName() + "/Best Target Yaw", bestTarget.getYaw());
                Logger.recordOutput(camera.getName() + "/Best Target Pitch", bestTarget.getPitch());

                Pose3d bestRobotPose = new Pose3d();

                if (tagFieldLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
                    bestRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            bestTarget.getBestCameraToTarget(),
                            tagFieldLayout.getTagPose(bestTarget.getFiducialId()).get(),
                            camToRobot);
                }

                Logger.recordOutput(camera.getName() + "/Best Target Pose ", bestRobotPose);

                for (PhotonTrackedTarget target : targets) {


                    Pose3d robotPose = new Pose3d();

                    if (tagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                target.getBestCameraToTarget(),
                                tagFieldLayout.getTagPose(target.getFiducialId()).get(),
                                camToRobot);
                    }
                    Logger.recordOutput(camera.getName() + "/Targets/" +  target.getFiducialId() + "/robotPose", robotPose);
                    targetPoses.add(robotPose);
                }
                Pose3d averagePose = poseAverage(targetPoses);

                Logger.recordOutput(camera.getName() + "/AveragePose", averagePose);
                targetPoses.clear();
                return Optional.of(averagePose);
            }
        }
        return Optional.empty();
    }
    @Override
    public Pose3d poseAverage(List<Pose3d> listToAverage){
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

    @Override
    public Optional<Pose3d> photonPoseEstimator() {
        return Optional.empty();
    }
    @Override
    public void simulationInit(){

        visionSim.addAprilTags(
                tagFieldLayout
        );
        SimCameraProperties cameraProps = new SimCameraProperties();

        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(90));
        cameraProps.setFPS(20);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera,cameraProps);
        //rearCameraSim = new PhotonCameraSim(rearCam,cameraProps);




        visionSim.addCamera(cameraSim, camToRobot);
        //visionSim.addCamera(rearCameraSim, kRobotToCamFront);
    }

}
