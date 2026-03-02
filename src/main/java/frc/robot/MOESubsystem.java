package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import lombok.extern.java.Log;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Optional;

public abstract class MOESubsystem<SensorType extends LoggableInputs> extends SubsystemBase implements LoggableInputs {

    @Getter
    @Setter
    private SensorType sensors;
    public String inputsKey;
    public Optional<MOESimulator> simulator = Optional.empty();
    private boolean sensorsProcessed = false;

    public MOESubsystem(SensorType sensors) {
        this(sensors, sensors.getClass().getName());
    }

    public MOESubsystem(SensorType sensors, String inputsKey) {
        this.sensors = sensors;
        this.inputsKey = inputsKey;
        System.out.println("Constructed Subsystem type: " + getClass());
    }

    @Override
    public void periodic() {
        this.readSensors(this.sensors);
        sensorsProcessed = true;
    }

    public void setSimulator(MOESimulator simulator) {
        this.simulator = Optional.of(simulator);
    }

    @Override
    public void simulationPeriodic() {
        simulator.ifPresent(MOESimulator::updateSimState);
    }

    public void readSensors(SensorType sensors) {
    }

    @Override
    public void toLog(LogTable table) {
        if (sensorsProcessed) sensors.toLog(table);
    }

    @Override
    public void fromLog(LogTable table) {
        sensors.fromLog(table);
    }
    
}
