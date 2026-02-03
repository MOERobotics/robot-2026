package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Optional;

public interface PhotonCameraSubsystem extends Subsystem, LoggableInputs {
    // Change this to match the name of your camera

    @AutoLog
    class VisionInputs {
        public boolean hasTargets = false;

        public int bestFiducialId = -1;
        public double bestYaw = 0.0;
        public double bestPitch = 0.0;
        public double bestArea = 0.0;
        public Pose3d estimatedPose = Pose3d.kZero;
        public boolean hasMultiEstimatedPose = false;


    }

    VisionInputs getSensors();



    default boolean hasTargets() {
        return getSensors().hasTargets;
    }


    default Optional<Pose3d> getEstimatedPose() {
            return Optional.ofNullable(getSensors().estimatedPose);

    }









}
