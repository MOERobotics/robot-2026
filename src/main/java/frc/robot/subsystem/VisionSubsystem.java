package frc.robot.subsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.List;
import java.util.Optional;

public interface VisionSubsystem extends Subsystem, LoggableInputs {


    Optional<Pose3d> photonFunction();

    Pose3d poseAverage(List<Pose3d> listToAverage);

    @AutoLog
    class CameraInputs {
/*
         boolean hasTargets;
         double yaw;
         double pitch;
         double area;
         int id;
         Transform3d cameraToRobot;

 */

             }
    public Optional<Pose3d> photonPoseEstimator();

}
