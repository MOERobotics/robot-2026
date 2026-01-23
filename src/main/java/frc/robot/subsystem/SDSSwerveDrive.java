package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SDSSwerveDrive implements SwerveDriveSubsystem {

    SwerveModule[] swerveModules;
    public SDSSwerveDrive(){

        swerveModules = new SDSSwerveModule[4];
        swerveModules[0] = new SDSSwerveModule();
        swerveModules[1] = new SDSSwerveModule();
        swerveModules[2] = new SDSSwerveModule();
        swerveModules[3] = new SDSSwerveModule();
    }

    @Override
    public SwerveModule[] getModules() {

    }

    @Override
    public void setChassisSpeed(ChassisSpeeds robotChassisSpeed) {

    }

    @Override
    public void setModuleStates(SwerveModuleState... robotModuleStates) {

    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public void setPose(Pose2d robotPose2D) {

    }

    @Override
    public ChassisSpeeds getChassisSpeed() {
        return null;
    }
}
