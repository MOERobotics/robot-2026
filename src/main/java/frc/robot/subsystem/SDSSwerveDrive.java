package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.SwerveModuleSubsystem;

import frc.robot.subsystem.simulations.SwerveDriveSim;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;

public class SDSSwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> implements SwerveDriveSubsystem {
    public SwerveDriveKinematics robotKinematics;
    SwerveModuleSubsystem[] swerveModules;
    SwerveDrivePoseEstimator robotOdometry;
    public Pigeon2 robotGyro;
    PhotonCamera turretCam = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");

    PhotonCamera swerveCam = new PhotonCamera("Arducam_OV9281_USB_Camera (3)");

    Transform3d turretCamLocation = new Transform3d(
            Millimeters.of(-272.98),
            Millimeters.of(317.26),
            Millimeters.of(180.0),
            new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-15),
                    Degrees.of(180)
            )
    );
    Transform3d swerveCamLocation = new Transform3d(
            Inches.of(-11.7459),
            Inches.of(-11.791),
            Inches.of(8.021),
            new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-27),
                    Degrees.of(225)
            )
    );

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    PhotonPoseEstimator estimator1 = new PhotonPoseEstimator(fieldLayout, turretCamLocation);
    PhotonPoseEstimator estimator2 = new PhotonPoseEstimator(fieldLayout, swerveCamLocation);

    // Pigeon2SimState simGyro;


    public SDSSwerveDrive(Pigeon2 robotGyro, SwerveModuleSubsystem... swerveModules) {
        super(new SwerveDriveInputsAutoLogged());
        this.robotGyro = robotGyro;
        this.swerveModules = swerveModules;
       // simGyro = robotGyro.getSimState();
        robotKinematics = new SwerveDriveKinematics(
                Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getCoordsOfModule).toArray(Translation2d[]::new)
        );
        robotOdometry = new SwerveDrivePoseEstimator(robotKinematics,
                robotGyro.getRotation2d(),
                Arrays.stream(swerveModules).map(SwerveModuleSubsystem::getTravelDistanceNRobotAngle).toArray(SwerveModulePosition[]::new),
                new Pose2d()
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
                        new PIDConstants(3, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
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
        turretCam.setFPSLimit(12);
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
        boolean rejectUpdate = false;
        photonPoses();

        if(getSensors().photon1 == null){
            rejectUpdate = true;
        }
        if (getSensors().photon2 == null){
            rejectUpdate = true;
        }
        if(robotGyro.getAngularVelocityZWorld().getValue().abs(DegreesPerSecond)>=360){
            rejectUpdate = true;
        }

        Logger.recordOutput("rejectUpdate", rejectUpdate);

        if(!rejectUpdate){
            this.robotOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.9,0.9,999999));
            if(getSensors().turretCamPose != null) {
                this.robotOdometry.addVisionMeasurement(getSensors().turretCamPose.toPose2d(), getSensors().photon1.getTimestampSeconds());
            }
            if(getSensors().swerveCamPose != null) {
                this.robotOdometry.addVisionMeasurement(getSensors().swerveCamPose.toPose2d(), getSensors().photon2.getTimestampSeconds());
            }
        }
    }


    @Override
    public SwerveModuleSubsystem[] getModules() {
        return swerveModules;
    }

    @Override
    public void robotDrive(ChassisSpeeds robotChassisSpeed, boolean robotCentric) {
        if(!robotCentric){
            robotChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(robotChassisSpeed,robotGyro.getRotation2d());
        }
        getSensors().sensorsChassisSpeeds = robotChassisSpeed;

        SwerveModuleState[] robotModuleStateToChassisSpeed = robotKinematics.toSwerveModuleStates(robotChassisSpeed);
        this.setModuleStates(robotModuleStateToChassisSpeed);
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
        return robotOdometry.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d robotPose2D) {

        if (DriverStation.isEnabled() && DriverStation.isTeleop()) {
            try {
                throw new RuntimeException("??????");
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }

        robotGyro.setYaw(robotPose2D.getRotation().getDegrees());

        robotOdometry.resetPosition(robotPose2D.getRotation(),
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

    @Override
    public void photonPoses(){
        List<PhotonPipelineResult> results1 = turretCam.getAllUnreadResults();
        if (!results1.isEmpty()) {
            Logger.recordOutput(
                    "turretCamResults",
                    PhotonPipelineResult.proto,
                    results1.get(results1.size()-1)
            );
            getSensors().photon1 = results1.get(results1.size()-1);
        }

        List<PhotonPipelineResult> results2 = swerveCam.getAllUnreadResults();

        if (!results2.isEmpty()) {
            Logger.recordOutput(
                    "swerveCamResults",
                    PhotonPipelineResult.proto,
                    results2.get(results2.size()-1)
            );
            getSensors().photon2 = results2.get(results2.size()-1);
        }
        getSensors().turretCamPose = (
                estimator1.estimateCoprocMultiTagPose(getSensors().photon1).
                        map((erp) -> erp.estimatedPose).
                        orElse(estimator1.estimateClosestToCameraHeightPose(getSensors().photon1).
                        map((erp) -> erp.estimatedPose).orElse(null))
        );
        var photonTargetSwerve = getSensors().photon2.getBestTarget();
        Pose3d photonBestSwerve = null;
        if (photonTargetSwerve != null) {
            photonBestSwerve = PhotonUtils.estimateFieldToRobotAprilTag(
                swerveCamLocation,
                fieldLayout.getTagPose(photonTargetSwerve.fiducialId).get(),
                photonTargetSwerve.bestCameraToTarget
            );
        }
        getSensors().swerveCamPose = (
                estimator2.estimateCoprocMultiTagPose(getSensors().photon2).
                        map((erp) -> erp.estimatedPose).
                        orElse(photonBestSwerve)
        );
    }


}
