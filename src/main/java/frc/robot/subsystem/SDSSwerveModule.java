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

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;

public class SDSSwerveModule extends MOESubsystem <SwerveModuleInputsAutoLogged> implements SwerveModule {

    private final SparkMax pivotMotor;
    private final SparkMax driveMotor;
    private final CANcoder swerveModuleEncoder;
    public Distance xCordinate;
    public Distance yCordinate;
    PIDController pidPivotController;
    PIDController pidDriveController;


    public RelativeEncoder driveMotorEncoder;
    public RelativeEncoder pivotMotorEncoder;
    SparkMaxSim pivotMotorSimulator, driveMotorSimulator;
    public SparkRelativeEncoderSim pivotMotorEncoderSimulator;
    public SparkRelativeEncoderSim driveMotorEncoderSimulator;
    public DCMotorSim pivotMotorSystem;
    public DCMotorSim driveMotorSystem;
    public CANcoderSimState pivotEncoderSim;
    public PIDConstants pivotFeedback;
    public PIDConstants driveFeedback;





    // SparkRelativeEncoderSim pivotMotorEncoder, driveMotorEncoder;

    public SDSSwerveModule (
        SparkMax driveMotor,
        SparkMax pivotMotor,
        CANcoder swerveModuleEncoder,
        Distance xCordinate,
        Distance yCordinate,
        PIDConstants pivotFeedback,
        PIDConstants driveFeedback
    ) {
        super(new SwerveModuleInputsAutoLogged());
        this.driveMotor = driveMotor;
        this.driveMotorEncoder = driveMotor.getEncoder();
        this.pivotMotor = pivotMotor;
        this.pivotMotorEncoder = pivotMotor.getEncoder();
        this.swerveModuleEncoder = swerveModuleEncoder;
        this.xCordinate = xCordinate;
        this.yCordinate = yCordinate;
        this.pivotEncoderSim = swerveModuleEncoder.getSimState();
        this.pivotFeedback = pivotFeedback;
        this.driveFeedback= driveFeedback;
        pivotMotorSimulator = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
        driveMotorSimulator = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));

        pivotMotorEncoderSimulator = pivotMotorSimulator.getRelativeEncoderSim();
        driveMotorEncoderSimulator = driveMotorSimulator.getRelativeEncoderSim();

        pivotMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25.0
                ),
                DCMotor.getNEO(1)
        );

        driveMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25.0
                ),
                DCMotor.getNEO(1)
        );
        pidPivotController = new PIDController(pivotFeedback.kP, pivotFeedback.kI,pivotFeedback.kD);
        pidDriveController = new PIDController(driveFeedback.kP, driveFeedback.kI, driveFeedback.kD);

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
    public SwerveModuleState getSpeedNDirectionOfMod() {
        return new SwerveModuleState(InchesPerSecond.of(driveMotor.getEncoder().getVelocity()*(4* PI / (60.0*6.75))).in(MetersPerSecond),
                new Rotation2d(getAngle()));
    }

        public Angle getAngle() {
            return this.swerveModuleEncoder.getPosition().getValue();
        }


    @Override
    public void setSpeed(double moduleSpeed) {
        driveMotor.set(-moduleSpeed);
    }

    @Override

    public void setPivot(Rotation2d modulePivot) {
        Angle currentWheelDirection = swerveModuleEncoder.getPosition().getValue();
        Angle wheelTargetDirection = modulePivot.getMeasure();
        Angle wheelError = currentWheelDirection.minus(wheelTargetDirection);
        pivotMotor.set(pidPivotController.calculate(wheelError.in(Degree)));
    }

    @Override
    public Translation2d getCoordsOfModule() {
        return new Translation2d(this.xCordinate, this.yCordinate);

    }

    @Override
    public SwerveModulePosition getTravelDistanceNRobotAngle() {

        return new SwerveModulePosition(
                Inches.of(
                driveMotor.getEncoder().getPosition()*(4* PI / 6.75)
                ).in(Meters),
                new Rotation2d(
                    getAngle()
                )
        );
    }
    public void simulate(){
        driveMotorSystem.setInputVoltage(driveMotor.getBusVoltage()*driveMotor.get());
        pivotMotorSystem.setInputVoltage(-pivotMotor.getBusVoltage()*pivotMotor.get());
        driveMotorSystem.setAngularVelocity(MOESimulator.decelerate(driveMotorSystem.getAngularVelocity(),60).in(RadiansPerSecond));
        pivotMotorSystem.setAngularVelocity(MOESimulator.decelerate(pivotMotorSystem.getAngularVelocity(),60).in(RadiansPerSecond));

        driveMotorSystem.update(.02);
        pivotMotorSystem.update(.02);

        driveMotorSimulator.iterate(driveMotorSystem.getAngularVelocityRPM()*6.75, 12.0, .02);
        pivotMotorSimulator.iterate(-pivotMotorSystem.getAngularVelocityRPM()*(150.0/7.0), 12.0, .02);
        pivotEncoderSim.setRawPosition(pivotMotorSystem.getAngularPosition().unaryMinus());
        pivotEncoderSim.setVelocity(pivotMotorSystem.getAngularVelocity().unaryMinus());
    }

    @Override
    public void simulationPeriodic() {
        simulate();
    }
}


