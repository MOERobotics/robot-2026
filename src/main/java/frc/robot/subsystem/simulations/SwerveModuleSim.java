package frc.robot.subsystem.simulations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESimulator;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.MOESimulator.decelerate;

public class SwerveModuleSim implements MOESimulator {
    private final double pivotReduction = 150.0/7.0;
    private final double driveReduction = 6.75;
    private final double decelerationCoef = 60;
    private final SparkMaxSim driveMotorSim, pivotMotorSim;
    private final SparkMax driveMotor, pivotMotor;
    private final CANcoderSimState pivotEncoderSim;
    private final DCMotorSim driveMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, driveReduction), DCMotor.getNEO(1));
    private final DCMotorSim pivotMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, pivotReduction), DCMotor.getNEO(1));
    private final Angle offset;

    public SwerveModuleSim(Angle offset, SparkMax driveMotor, SparkMax pivotMotor, CANcoder pivotEncoder) {
        this.offset = offset;
        this.driveMotor = driveMotor;
        this.pivotMotor = pivotMotor;
        this.driveMotorSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));
        this.pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
        this.pivotEncoderSim = pivotEncoder.getSimState();
    }

    public void updateSimState() {
        driveMotorSystem.setInputVoltage(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
        driveMotorSystem.setAngularVelocity(decelerate(driveMotorSystem.getAngularVelocity(),decelerationCoef).in(RadiansPerSecond));
        pivotMotorSystem.setInputVoltage(-pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
        pivotMotorSystem.setAngularVelocity(decelerate(pivotMotorSystem.getAngularVelocity(),decelerationCoef).in(RadiansPerSecond));

        driveMotorSystem.update(.02);
        pivotMotorSystem.update(.02);

        driveMotorSim.iterate(driveMotorSystem.getAngularVelocityRPM()*driveReduction, 12.0, 0.02);
        pivotMotorSim.iterate(-pivotMotorSystem.getAngularVelocityRPM()*pivotReduction, 12.0, 0.02);
        pivotMotorSim.getAlternateEncoderSim().iterate(pivotMotorSystem.getAngularVelocityRPM()*pivotReduction, .02);
        pivotEncoderSim.setRawPosition(pivotMotorSystem.getAngularPosition().unaryMinus().minus(offset));
        pivotEncoderSim.setVelocity(pivotMotorSystem.getAngularVelocity().unaryMinus());
    }


}