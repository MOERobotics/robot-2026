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
import edu.wpi.first.wpilibj.AnalogInput;
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
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;

public class SDSSwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> implements SwerveDriveSubsystem {
    public SwerveDriveKinematics robotKinematics;
    SwerveModuleSubsystem[] swerveModules;
    SwerveDrivePoseEstimator robotOdometry;
    public Pigeon2 robotGyro;
    PhotonCamera turretCam = new PhotonCamera("turretCamera");

    PhotonCamera swerveCam = new PhotonCamera("swerveCamera");

    public static final double MAX_DISTANCE = 5.0;
    public static final double MAX_AMBIGUITY = 0.25;


    private AnalogInput kevin_pi_voltage_monitor = new AnalogInput(3);

    double num = 2;

    Transform3d turretCamLocation = new Transform3d(
            Inches.of(-10.7472441),
            Inches.of(12.4905512),
            Inches.of(8.996732),
            new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-15),
                    Degrees.of(180)
            )
    );
    Transform3d swerveCamLocation = new Transform3d(
            Inches.of(-11.745944),
            Inches.of(-11.791497),
            Inches.of(8.021322),
            new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(-27),
                    Degrees.of(225)
            ).unaryMinus()
//            new Rotation3d(
//                    Degrees.of(20.5),
//                    Degrees.of(0),
//                    Degrees.of(0)
//            ).rotateBy(new Rotation3d(
//                    Degrees.of(0),
//                    Degrees.of(-23),
//                    Degrees.of(0)
//            )).rotateBy(
//                    new Rotation3d(
//                            Degrees.of(0),
//                            Degrees.of(0),
//                            Degrees.of(135)
//                    )
//            )
    );

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    PhotonPoseEstimator estimator1 = new PhotonPoseEstimator(fieldLayout, turretCamLocation);
    PhotonPoseEstimator estimator2 = new PhotonPoseEstimator(fieldLayout, swerveCamLocation);


    double diffAngleAvg = 0;

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
        try {
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
                        new PIDConstants(2, 0.0, 0.0), // Translation PID constants
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
        diffAngleAvg = robotGyro.getRotation2d().getRadians();


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

        /*
        if (getSensors().photon1 == null && getSensors().photon2 == null) {
            rejectUpdate = true;
        }


        if (robotGyro.getAngularVelocityZWorld().getValue().abs(DegreesPerSecond) >= 360) {
            rejectUpdate = true;
        }


         */
        Logger.recordOutput("kevin pi voltage", kevin_pi_voltage_monitor.getVoltage());

        Logger.recordOutput("rejectUpdate", rejectUpdate);

        if (getSensors().photon1 != null) {
            Logger.recordOutput("photon1Timestamp", getSensors().photon1.getTimestampSeconds());

        }

        if (getSensors().photon2 != null) {
            Logger.recordOutput("photon2Timestamp", getSensors().photon2.getTimestampSeconds());

        }

        // decide how much we wanna trust cams
        this.robotOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 5));

        // make sure cams work and decide if we trust to add them
        addVisionMeasurements(getSensors().photon1, turretCamLocation, "turret");
        addVisionMeasurements(getSensors().photon2, swerveCamLocation, "swerve");

        /*
        if (!rejectUpdate || true) {
            this.robotOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 5));
            if (getSensors().turretCamPose != null && getSensors().photon1 != null) {
                this.robotOdometry.addVisionMeasurement(getSensors().turretCamPose.toPose2d(), getSensors().photon1.getTimestampSeconds());
            }
            if (getSensors().swerveCamPose != null && getSensors().photon2 != null) {
                this.robotOdometry.addVisionMeasurement(getSensors().swerveCamPose.toPose2d(), getSensors().photon2.getTimestampSeconds());
            }
        }
         */

        // diffAngleAvg = (diffAngleAvg * ((num-1)/num)) + ((getPose().getRotation().getRadians()- getSensors().turretCamPose.getRotation().getAngle())/num);
        //  this.setPose(new Pose2d(getPose().getX(), getPose().getY(), (getPose().getRotation().minus(new Rotation2d(diffAngleAvg)))));

    }


    @Override
    public SwerveModuleSubsystem[] getModules() {
        return swerveModules;
    }

    @Override
    public void robotDrive(ChassisSpeeds robotChassisSpeed, boolean robotCentric) {
        if (!robotCentric) {
            robotChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(robotChassisSpeed, getPose().getRotation());
        }
        getSensors().sensorsChassisSpeeds = robotChassisSpeed;

        SwerveModuleState[] robotModuleStateToChassisSpeed = robotKinematics.toSwerveModuleStates(robotChassisSpeed);
        this.setModuleStates(robotModuleStateToChassisSpeed);
    }

    @Override
    public void brake() {
        swerveModules[0].brake();
        swerveModules[1].brake();
        swerveModules[2].brake();
        swerveModules[3].brake();
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
    public void photonPoses() {
        List<PhotonPipelineResult> results1 = turretCam.getAllUnreadResults();

        if (!results1.isEmpty()) {
            Logger.recordOutput(
                    "turretCamResults",
                    PhotonPipelineResult.proto,
                    results1.get(results1.size() - 1)
            );
            getSensors().photon1 = results1.get(results1.size() - 1);
            getSensors().hasNewPhoton1 = true;

        } else {
            getSensors().photon1 = null;
        }

        List<PhotonPipelineResult> results2 = swerveCam.getAllUnreadResults();

        if (!results2.isEmpty()) {
            Logger.recordOutput(
                    "swerveCamResults",
                    PhotonPipelineResult.proto,
                    results2.get(results2.size() - 1)
            );
            getSensors().photon2 = results2.get(results2.size() - 1);

        } else {
            getSensors().photon2 = null;
        }
    }

    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult photonPipelineResult) {

        List<PhotonTrackedTarget> targets = photonPipelineResult.targets;
        PhotonTrackedTarget bestTarget=null;

        for (PhotonTrackedTarget target : targets){
            if (bestTarget==null) {
                bestTarget= target;
                continue;
            }
            if(target.poseAmbiguity < bestTarget.poseAmbiguity){
                bestTarget = target;
            }
        }
        return bestTarget;
    }


    public void addVisionMeasurements(PhotonPipelineResult photonPipelineResult, Transform3d cameraPosition, String sensorPoseStr) {
        Pose3d outputPose = null;
        if (photonPipelineResult != null) {
            if (cameraPosition != null) {
                

                if (photonPipelineResult.hasTargets()) {

                    var photonBestTarget = getBestTarget(photonPipelineResult);
                    Logger.recordOutput("bestTarget", photonBestTarget);


                    if (photonBestTarget != null) {

                        Pose3d photonBestPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                photonBestTarget.bestCameraToTarget,
                                fieldLayout.getTagPose(photonBestTarget.fiducialId).get(),
                                cameraPosition
                        );

                        Logger.recordOutput("bestPose", photonBestPose);

                        double ambiguity = photonBestTarget.poseAmbiguity;
                        double distance = photonBestTarget.bestCameraToTarget.getTranslation().getNorm();
                        
                        if (distance > MAX_DISTANCE || ambiguity > MAX_AMBIGUITY) {
                            outputPose = null;
                        } else {
                            outputPose = photonBestPose;
                        }
                        if (sensorPoseStr.equals("turret")){
                            getSensors().turretCamPose = outputPose;
                        }
                        if (sensorPoseStr.equals("swerve")){
                            getSensors().swerveCamPose = outputPose;
                        }
                    }
                    }else {
                        photonPipelineResult = null;
                    }
                }

                if (photonPipelineResult != null) {
                    if (outputPose != null) {
                        if (robotGyro.getAngularVelocityZWorld().getValue().abs(DegreesPerSecond) < 360) {
                            this.robotOdometry.addVisionMeasurement(outputPose.toPose2d(), photonPipelineResult.getTimestampSeconds());
                        }
                    }
                }

            }
        }
    }


