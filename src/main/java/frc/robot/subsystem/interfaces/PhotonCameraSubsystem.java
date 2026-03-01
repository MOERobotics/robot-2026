package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Optional;

public interface PhotonCameraSubsystem extends Subsystem, LoggableInputs {
    Optional<Rotation2d> angleToTarget(int target);
    // Change this to match the name of your camera

    @AutoLog
    class VisionInputs {
        public boolean hasTargets = false;

        public int bestFiducialId = -1;
        public double bestYaw = 0.0;
        public double bestPitch = 0.0;
        public double bestArea = 0.0;
        public Pose3d estimatedPose = Pose3d.kZero;

        public boolean hasEstimatedPose = false;

        public Pose3d multiTagPose = Pose3d.kZero;
        public boolean hasMultiTagPose = false;

        public Pose3d singleTagPose = Pose3d.kZero;
        public boolean hasSingleTagPose = false;

        public Pose3d finalVisionPose = Pose3d.kZero;
        public String finalPoseSource = "NONE";




    }

    VisionInputs getSensors();



    default boolean hasTargets() {
        return getSensors().hasTargets;
    }

    default Optional<Pose3d> getEstimatedPose() {

        if( getSensors().hasEstimatedPose){
           return Optional.of(getSensors().estimatedPose);
        }else {
            return Optional.empty();
        }

    }

    default boolean hasMultiTagPose() {
        return getSensors().hasEstimatedPose;
    }

    default int getBestFiducialId() {
        return getSensors().bestFiducialId;
    }

    default double getBestYaw() {
        return getSensors().bestYaw;
    }

    default double getBestPitch() {
        return getSensors().bestPitch;
    }

    default double getBestArea() {
        return getSensors().bestArea;
    }



}
