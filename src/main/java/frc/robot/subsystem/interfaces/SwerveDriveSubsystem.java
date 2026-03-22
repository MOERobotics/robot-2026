package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.SwerveModuleSubsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface SwerveDriveSubsystem extends Subsystem, LoggableInputs {
    public SwerveModuleSubsystem[] getModules();

    public void robotDrive(ChassisSpeeds robotChassisSpeed, boolean robotCentric);

    public void setModuleStates(SwerveModuleState... robotModuleStates);

    public Pose2d getPose();

    public void setPose(Pose2d robotPose2D);

    public ChassisSpeeds getChassisSpeed();

    default void stop(){}

    default void photonPoses(){}


    @AutoLog
    public static class SwerveDriveInputs {
        public Angle robotAngle;
        public ChassisSpeeds robotChassisSpeed;
        public SwerveModuleSubsystem moduleFL;
        public SwerveModuleSubsystem moduleFR;
        public SwerveModuleSubsystem moduleBL;
        public SwerveModuleSubsystem moduleBR;
        public Pose2d robotPose2D;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] requestedModuleStates;
        public SwerveModulePosition[] modulePositions;
        public ChassisSpeeds sensorsChassisSpeeds;
        public PhotonPipelineResult photon1 = new PhotonPipelineResult();
        public PhotonPipelineResult photon2 = new PhotonPipelineResult();
        public Pose3d turretCamPose = new Pose3d();
        public Pose3d swerveCamPose = new Pose3d();

    }
}
