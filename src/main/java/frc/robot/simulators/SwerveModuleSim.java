package frc.robot.simulators;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESimulator;

import frc.robot.subsystem.SDSSwerveModule;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveModuleSim implements MOESimulator {

    SparkMaxSim pivotMotorSimulator, driveMotorSimulator;
    public SparkRelativeEncoderSim pivotMotorEncoderSimulator;
    public SparkRelativeEncoderSim driveMotorEncoderSimulator;
    public DCMotorSim pivotMotorSystem;
    public DCMotorSim driveMotorSystem;
    public CANcoderSimState pivotEncoderSim;
    public SDSSwerveModule swerveModule;


    public SwerveModuleSim(SDSSwerveModule swerveModule){
        this.swerveModule = swerveModule;
        pivotMotorSimulator = new SparkMaxSim(swerveModule.pivotMotor, DCMotor.getNEO(1));
        driveMotorSimulator = new SparkMaxSim(swerveModule.driveMotor, DCMotor.getNEO(1));

        pivotEncoderSim = swerveModule.swerveModuleEncoder.getSimState();

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
        );    }
    @Override
    public void updateSimState() {
        driveMotorSystem.setInputVoltage(swerveModule.driveMotor.getBusVoltage() * swerveModule.driveMotor.get());
        pivotMotorSystem.setInputVoltage(-swerveModule.pivotMotor.getBusVoltage() * swerveModule.pivotMotor.get());
        driveMotorSystem.setAngularVelocity(MOESimulator.decelerate(driveMotorSystem.getAngularVelocity(), 60).in(RadiansPerSecond));
        pivotMotorSystem.setAngularVelocity(MOESimulator.decelerate(pivotMotorSystem.getAngularVelocity(), 60).in(RadiansPerSecond));

        driveMotorSystem.update(.02);
        pivotMotorSystem.update(.02);

        driveMotorSimulator.iterate(driveMotorSystem.getAngularVelocityRPM() * 6.75, 12.0, .02);
        pivotMotorSimulator.iterate(-pivotMotorSystem.getAngularVelocityRPM() * (150.0 / 7.0), 12.0, .02);
        pivotEncoderSim.setRawPosition(pivotMotorSystem.getAngularPosition().unaryMinus());
        pivotEncoderSim.setVelocity(pivotMotorSystem.getAngularVelocity().unaryMinus());
    }
}
