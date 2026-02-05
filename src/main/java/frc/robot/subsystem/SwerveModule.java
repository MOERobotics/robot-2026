package frc.robot.subsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveModule extends Subsystem, LoggableInputs {
    public SwerveModuleState getSpeedNDirectionOfMod(); // check if state is correct
    public void setSpeed(double moduleSpeed);
    public void setPivot(Rotation2d modulePivot);
    public Translation2d getCoordsOfModule();
    public SwerveModulePosition getTravelDistanceNRobotAngle();
    public SwerveModuleInputsAutoLogged getSensors();

    @AutoLog
    public class SwerveModuleInputs {
        Angle moduleAngle;
        Translation2d robotTranslation;
        SwerveModulePosition robotPosition;
        SwerveModuleState robotModuleState;
        double robotPivotSpeed;
        double robotDriveSpeed;

    }
}


