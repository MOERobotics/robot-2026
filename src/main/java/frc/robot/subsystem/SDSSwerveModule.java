package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;

public class SDSSwerveModule extends MOESubsystem <SwerveModuleInputsAutoLogged> implements SwerveModule {

    private final SparkMax pivotMotor = null;
    private final SparkMax driveMotor = null;
    private final CANcoder swerveModuleEncoder;
    public double xCordinate;
    public double yCordinate;
    PIDController miniPivotPIDController = new PIDController(1/45.0,0,0);
    PIDController submoePivotPIDController = new PIDController(0.5,0,0);

    public RelativeEncoder driveMotorEncoder = driveMotor.getEncoder();
    public RelativeEncoder pivotMotorEncoder = pivotMotor.getEncoder();
    SparkMaxSim pivotMotorSimulator, driveMotorSimulator;
    // SparkRelativeEncoderSim pivotMotorEncoder, driveMotorEncoder;

    public SDSSwerveModule (
        SparkMax driveMotor,
        SparkMax pivotMotor,
        CANcoder swerveModuleEncoder,
        double xCordinate,
        double yCordinate
    ) {
        super(new SwerveModuleInputsAutoLogged());
        this.driveMotor = driveMotor;
        this.pivotMotor = pivotMotor;
        this.swerveModuleEncoder = swerveModuleEncoder;
        this.xCordinate = xCordinate;
        this.yCordinate = yCordinate;

    }

    @Override
    public void readSensors(SwerveModuleInputsAutoLogged sensors) {
        sensors.moduleAngle = swerveModuleEncoder.getPosition().getValue();
        sensors.robotDriveSpeed = driveMotor.get();
        sensors.robotModuleState = new SwerveModuleState(
                InchesPerSecond.of(driveMotor.getEncoder().getVelocity()*(4* PI / (60.0*6.75))).in(MetersPerSecond),
                new Rotation2d(getAngle())
        );
        sensors.robotPivotSpeed = pivotMotor.get();
        sensors.robotPosition = new SwerveModulePosition(
                Inches.of(
                        driveMotor.getEncoder().getPosition()*(4* PI / 6.75)
                ).in(Meters),
                new Rotation2d(
                        getAngle()
                )
        );
        sensors.robotTranslation = new Translation2d(this.xCordinate, this.yCordinate);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(InchesPerSecond.of(driveMotor.getEncoder().getVelocity()*(4* PI / (60.0*6.75))).in(MetersPerSecond),
                new Rotation2d(getAngle()));
    }

        public Angle getAngle() {
            return this.swerveModuleEncoder.getPosition().getValue();
        }


    @Override
    public void setSpeed(double moduleSpeed) {
        driveMotor.set(moduleSpeed);
    }

    @Override

    public void setPivot(Rotation2d modulePivot) {
        Angle currentWheelDirection = swerveModuleEncoder.getPosition().getValue();
        Angle wheelTargetDirection = modulePivot.getMeasure();
        Angle wheelError = currentWheelDirection.minus(wheelTargetDirection);
        pivotMotor.set(submoePivotPIDController.calculate(wheelError.in(Degree)));
    }

    @Override
    public Translation2d getTranslation() {
        return new Translation2d(this.xCordinate, this.yCordinate);

    }

    @Override
    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(
                Inches.of(
                driveMotor.getEncoder().getPosition()*(4* PI / 6.75)
                ).in(Meters),
                new Rotation2d(
                    getAngle()
                )
        );
    }}


