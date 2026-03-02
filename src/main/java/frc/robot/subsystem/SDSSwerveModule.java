package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESimulator;
import frc.robot.MOESubsystem;
import frc.robot.simulators.SwerveModuleSim;
import frc.robot.subsystem.interfaces.SwerveModuleInputsAutoLogged;
import frc.robot.subsystem.interfaces.SwerveModuleSubsystem;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;

public class SDSSwerveModule extends MOESubsystem<SwerveModuleInputsAutoLogged> implements SwerveModuleSubsystem {

    public final SparkMax pivotMotor;
    public final SparkMax driveMotor;
    public final CANcoder swerveModuleEncoder;
    public Distance xCordinate;
    public Distance yCordinate;
    PIDController pidPivotController;
    PIDController pidDriveController;


    public RelativeEncoder driveMotorEncoder;
    public RelativeEncoder pivotMotorEncoder;

    public CANcoderSimState pivotEncoderSim;
    public PIDConstants pivotFeedback;
    public PIDConstants driveFeedback;
    public final Angle moduleOffset;

    public SwerveModuleSim swerveModuleSim;


    // SparkRelativeEncoderSim pivotMotorEncoder, driveMotorEncoder;

    public SDSSwerveModule(
            SparkMax driveMotor,
            SparkMax pivotMotor,
            CANcoder swerveModuleEncoder,
            Distance xCordinate,
            Distance yCordinate,
            PIDConstants pivotFeedback,
            PIDConstants driveFeedback,
            Angle moduleOffset //BOLT ON RIGHT
    ) {
        super(new SwerveModuleInputsAutoLogged());
        this.driveMotor = driveMotor;
        this.driveMotorEncoder = driveMotor.getEncoder();
        this.pivotMotor = pivotMotor;
        this.pivotMotorEncoder = pivotMotor.getEncoder();
        this.swerveModuleEncoder = swerveModuleEncoder;
        this.xCordinate = xCordinate;
        this.yCordinate = yCordinate;
        this.moduleOffset = moduleOffset;

        this.pivotFeedback = pivotFeedback;
        this.driveFeedback = driveFeedback;

        pidPivotController = new PIDController(pivotFeedback.kP, pivotFeedback.kI, pivotFeedback.kD);
        pidDriveController = new PIDController(driveFeedback.kP, driveFeedback.kI, driveFeedback.kD);

        swerveModuleSim = new SwerveModuleSim(this);

        setSimulator(swerveModuleSim);

    }

    @Override
    public void readSensors(SwerveModuleInputsAutoLogged sensors) {

        sensors.moduleAngle = getAngle();
        sensors.robotDriveSpeed = driveMotor.get();
        sensors.speedNDirectionOfMod = new SwerveModuleState(
                InchesPerSecond.of(driveMotor.getEncoder().getVelocity() * (4 * PI / (60.0 * 6.75))).in(MetersPerSecond),
                new Rotation2d(getAngle())
        );
        sensors.robotPivotSpeed = pivotMotor.get();
        sensors.travelDistanceNRobotAngle = new SwerveModulePosition(
                Inches.of(
                        driveMotor.getEncoder().getPosition() * (4 * PI / 6.75)
                ).in(Meters),
                new Rotation2d(
                        getAngle()
                )
        );
        sensors.coordOfModule = new Translation2d(this.xCordinate, this.yCordinate);
    }

    @Override
    public SwerveModuleState getSpeedNDirectionOfMod() {
        return new SwerveModuleState(InchesPerSecond.of(driveMotor.getEncoder().getVelocity() * (4 * PI / (60.0 * 6.75))).in(MetersPerSecond),
                new Rotation2d(getAngle()));
    }

    public Angle getAngle() {
        return this.swerveModuleEncoder.getAbsolutePosition(true).getValue().minus(moduleOffset); //change back from absolute
    }


    @Override
    public void setSpeed(double moduleSpeed) {
        driveMotor.set(moduleSpeed);
    }

    @Override

    public void setPivot(Rotation2d modulePivot) {
        Angle currentWheelDirection = getAngle();
        Angle wheelTargetDirection = modulePivot.getMeasure();
        this.getSensors().moduleTargetAngle = wheelTargetDirection;
        Angle wheelError = currentWheelDirection.minus(wheelTargetDirection);
        double robotError = pidPivotController.calculate(wheelError.in(Degree));
        pivotMotor.set(robotError);
        this.getSensors().robotError = robotError;
    }

    @Override
    public Translation2d getCoordsOfModule() {
        return new Translation2d(this.xCordinate, this.yCordinate);

    }

    @Override
    public SwerveModulePosition getTravelDistanceNRobotAngle() {

        return new SwerveModulePosition(
                Inches.of(
                        driveMotor.getEncoder().getPosition() * (4 * PI / 6.75)
                ).in(Meters),
                new Rotation2d(
                        getAngle()
                )
        );
    }

}


