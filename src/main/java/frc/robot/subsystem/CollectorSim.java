package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESimulator;

import static edu.wpi.first.units.Units.*;

public class CollectorSim implements MOESimulator {
    private final SparkMax armMotor, wheelMotor;
    private final SparkMaxSim armMotorSim, wheelMotorSim;
    private final DCMotorSim armMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1.0), DCMotor.getNEO(1));
    private final DCMotorSim wheelMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1.0), DCMotor.getNEO(1));
    public final SparkRelativeEncoderSim armMotorEncoderSimulator;
    public final SparkRelativeEncoderSim wheelMotorEncoderSimulator;

    public CollectorSim(SparkMax armMotor, SparkMax wheelMotor, SparkMaxSim armMotorSim, SparkMaxSim wheelMotorSim) {
        this.armMotor = armMotor;
        this.wheelMotor = wheelMotor;
        this.armMotorSim = armMotorSim;
        this.wheelMotorSim = wheelMotorSim;

       this.armMotorEncoderSimulator = new SparkRelativeEncoderSim(armMotor);
       this.wheelMotorEncoderSimulator = new SparkRelativeEncoderSim(wheelMotor);


    }

    @Override
    public void updateSimState() {
        armMotorSystem.setInputVoltage(armMotor.getBusVoltage() * armMotor.get());
        wheelMotorSystem.setInputVoltage(-wheelMotor.getBusVoltage() * wheelMotor.get());
        armMotorSystem.setAngularVelocity(MOESimulator.decelerate(armMotorSystem.getAngularVelocity(), 60).in(RadiansPerSecond));
        wheelMotorSystem.setAngularVelocity(MOESimulator.decelerate(wheelMotorSystem.getAngularVelocity(), 60).in(RadiansPerSecond));

        armMotorSystem.update(.02);
        wheelMotorSystem.update(.02);

        armMotorEncoderSimulator.setPosition(armMotorSystem.getAngularPosition().in(Rotations));
        armMotorEncoderSimulator.setVelocity(armMotorSystem.getAngularVelocity().unaryMinus().in(RotationsPerSecond));
        wheelMotorEncoderSimulator.setVelocity(wheelMotorSystem.getAngularVelocity().unaryMinus().in(RotationsPerSecond));
    }

    @Override
    public void simulationPeriodic() {
        updateSimState();
    }



}