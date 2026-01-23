package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDriveSubsystem {
    public SwerveModule[] getModules ();
    public void setChassisSpeed(ChassisSpeeds robotChassisSpeed);
    public void setModuleStates(SwerveModuleState... robotModuleStates);
    public Pose2d getPose();
    public void setPose(Pose2d robotPose2D);
    public ChassisSpeeds getChassisSpeed();

    }
