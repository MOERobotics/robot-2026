package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.SwerveModuleSubsystem;

import frc.robot.subsystem.simulations.SwerveDriveSim;
import java.util.Arrays;

public class SDSSwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> implements SwerveDriveSubsystem {
    public SwerveDriveKinematics robotKinematics;
    SwerveModuleSubsystem[] swerveModules;
    SwerveDriveOdometry robotOdometry;
    public Pigeon2 robotGyro;
    // Pigeon2SimState simGyro;


    public SDSSwerveDrive(Pigeon2 robotGyro, SwerveModuleSubsystem... swerveModules) {
        super(new SwerveDriveInputsAutoLogged());
        this.robotGyro = robotGyro;
        this.swerveModules = swerveModules;
       // simGyro = robotGyro.getSimState();
        robotKinematics = new SwerveDriveKinematics(
                Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getCoordsOfModule).toArray(Translation2d[]::new)
        );
        robotOdometry = new SwerveDriveOdometry(robotKinematics,
                robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getTravelDistanceNRobotAngle).toArray(SwerveModulePosition[]::new)
        );
        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> robotDrive(speeds, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        SwerveDriveSim swerveSim = new SwerveDriveSim(this);
        setSimulator(swerveSim);


    }

    @Override
    public void readSensors(SwerveDriveInputsAutoLogged sensors) {
        sensors.robotAngle = robotGyro.getRotation2d().getMeasure();
        sensors.robotChassisSpeed = robotKinematics.toChassisSpeeds(Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getSpeedNDirectionOfMod).toArray(SwerveModuleState[]::new));
        sensors.robotPose2D = robotOdometry.update(robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getTravelDistanceNRobotAngle).toArray(SwerveModulePosition[]::new));
        sensors.moduleFL = swerveModules[0];
        sensors.moduleFR = swerveModules[1];
        sensors.moduleBL = swerveModules[2];
        sensors.moduleBR = swerveModules[3];
        sensors.modulePositions = Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getTravelDistanceNRobotAngle).toArray(SwerveModulePosition[]::new);
        sensors.moduleStates = Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getSpeedNDirectionOfMod).toArray(SwerveModuleState[]::new);
    }


    @Override
    public SwerveModuleSubsystem[] getModules() {
        return swerveModules;
    }

    @Override
    public void robotDrive(ChassisSpeeds robotChassisSpeed, boolean robotCentric) {

        if(!robotCentric){
            robotChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(robotChassisSpeed, this.getPose().getRotation());
        }

        SwerveModuleState[] robotModuleStateToChassisSpeed = robotKinematics.toSwerveModuleStates(robotChassisSpeed);
        this.setModuleStates(robotModuleStateToChassisSpeed);
    }

    public void simulate() {

    }
    @Override
    public void setModuleStates(SwerveModuleState... robotModuleStates) {
        this.getSensors().requestedModuleStates = robotModuleStates;
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModuleState moduleState = robotModuleStates[i];
            moduleState.optimize(swerveModules[i].getTravelDistanceNRobotAngle().angle);

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

        robotGyro.setYaw(robotPose2D.getRotation().getDegrees());

        robotOdometry.resetPosition(robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getTravelDistanceNRobotAngle).toArray(SwerveModulePosition[]::new),
                robotPose2D);
    }


    @Override
    public void stop() {
        for (SwerveModuleSubsystem module : swerveModules) {
            module.stop();
        }

    }

    @Override
    public ChassisSpeeds getChassisSpeed() {
        return robotKinematics.toChassisSpeeds(Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getSpeedNDirectionOfMod).toArray(SwerveModuleState[]::new));
    }

}
