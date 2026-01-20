package frc.robot.subsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public SwerveModuleState getState();
    public void setSpeed(double moduleSpeed);
    public void setPivot(Rotation2d modulePivot);
    public Translation2d getTranslation();
    public SwerveModulePosition getPosition();

    }

