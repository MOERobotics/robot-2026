package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveDriveSubsystem extends Subsystem, LoggableInputs {
    public SwerveModule[] getModules ();
    public void setChassisSpeed(ChassisSpeeds robotChassisSpeed);
    public void setModuleStates(SwerveModuleState... robotModuleStates);
    public Pose2d getPose();
    public void setPose(Pose2d robotPose2D);
    public ChassisSpeeds getChassisSpeed();

    @AutoLog
    public static class SwerveDriveInputs {
        Angle robotAngle;
        ChassisSpeeds robotChassisSpeed;
        SwerveModule moduleFL;
        SwerveModule moduleFR;
        SwerveModule moduleBL;
        SwerveModule moduleBR;
        Pose2d robotPose2D;

    }
    }
