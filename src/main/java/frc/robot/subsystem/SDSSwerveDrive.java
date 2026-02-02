package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.MOESubsystem;

import java.util.Arrays;

public class SDSSwerveDrive extends MOESubsystem <SwerveDriveInputsAutoLogged> implements SwerveDriveSubsystem {
SwerveDriveKinematics robotKinematics;
    SwerveModule[] swerveModules;
    SwerveDriveOdometry robotOdometry;
    Pigeon2 robotGyro;
    public SDSSwerveDrive(Pigeon2 robotGyro,SwerveModule... swerveModules){
        super(new SwerveDriveInputsAutoLogged());
        this.robotGyro = robotGyro;
        this.swerveModules = swerveModules;
        robotKinematics = new SwerveDriveKinematics(
                Arrays.stream(swerveModules).map(SwerveModule::getTranslation).toArray(Translation2d[]::new)
        );
        robotOdometry = new SwerveDriveOdometry(robotKinematics,
                robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new)
        );
    }

    @Override
    public void readSensors(SwerveDriveInputsAutoLogged sensors) {
        sensors.robotAngle = robotGyro.getRotation2d().getMeasure();
        sensors.robotChassisSpeed = robotKinematics.toChassisSpeeds(Arrays.stream(swerveModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new));
        sensors.robotPose2D = robotOdometry.update(robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new));
        sensors.moduleFL = swerveModules[0];
        sensors.moduleFR = swerveModules[1];
        sensors.moduleBL = swerveModules[2];
        sensors.moduleBR = swerveModules[3];
        sensors.modulePositions = Arrays.stream(swerveModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
        sensors.moduleStates = Arrays.stream(swerveModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    @Override
    public SwerveModule[] getModules() {
        return swerveModules;
    }

    @Override
    public void robotDrive(ChassisSpeeds robotChassisSpeed) {
        SwerveModuleState[] robotModuleStateToChassisSpeed = robotKinematics.toSwerveModuleStates(robotChassisSpeed);
        this.setModuleStates(robotModuleStateToChassisSpeed);
    }

    @Override
    public void setModuleStates(SwerveModuleState... robotModuleStates) {
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModuleState moduleState = robotModuleStates[i];
            moduleState.optimize(swerveModules[i].getPosition().angle);

            swerveModules[i].setPivot(moduleState.angle);
            swerveModules[i].setSpeed(moduleState.speedMetersPerSecond);
        }
    }

    @Override
    public Pose2d getPose() {
        return robotOdometry.getPoseMeters();
    }

    @Override
    public void setPose(Pose2d robotPose2D) {
        robotOdometry.resetPosition(robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new),
                robotPose2D);
    }

    @Override
    public ChassisSpeeds getChassisSpeed() {
        return robotKinematics.toChassisSpeeds(Arrays.stream(swerveModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new));
    }
}
