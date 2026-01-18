package frc.robot.subsystem;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PhotonCameraSubsystem extends Subsystem, LoggableInputs {

    @AutoLog
    class CameraInputs {

    }

}
