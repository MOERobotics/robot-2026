package frc.robot.subsystem.simulators;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESimulator;
import frc.robot.subsystem.Climber;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class ClimberSim implements MOESimulator {
    private final SparkMax climberSparkmax;
    private final DCMotorSim climberSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(
                    1),
            0.001,
            1.0),
            DCMotor.getNeo550(1));
    private final SparkMaxSim climberMotorSim;

    public ClimberSim(Climber climber) {
        this.climberSparkmax = climber.climberSparkMax;
        this.climberMotorSim = new SparkMaxSim(climberSparkmax, DCMotor.getNeo550(1));

    }

    @Override
    public void updateSimState() {
        climberSystem.setInputVoltage(climberSparkmax.getBusVoltage() * climberSparkmax.getAppliedOutput());
        climberSystem.setAngularVelocity
                (MOESimulator.decelerate(climberSystem.getAngularVelocity(),
                        60).in(RadiansPerSecond));

        climberSystem.update(0.02);


        climberMotorSim.iterate(climberSystem.getAngularVelocityRPM(), 12.0, 0.02);

    }

    @Override
    public void simulationPeriodic() {

    }
}
