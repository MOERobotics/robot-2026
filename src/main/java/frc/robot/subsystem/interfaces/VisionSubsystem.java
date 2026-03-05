package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.List;
import java.util.Optional;

public interface VisionSubsystem extends Subsystem, LoggableInputs {


    Optional<Pose3d> photonFunction();

    Pose3d poseAverage(List<Pose3d> listToAverage);

    @AutoLog
    class CameraInputs {
        //Optional<Pose3d> averagePose;
             }
    public Optional<Pose3d> photonPoseEstimator();

}
