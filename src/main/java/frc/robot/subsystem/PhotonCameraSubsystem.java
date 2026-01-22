package frc.robot.subsystem;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;

public interface PhotonCameraSubsystem extends Subsystem, LoggableInputs {
    // Change this to match the name of your camera

    @AutoLog
    class CameraInputs {

    }

}
