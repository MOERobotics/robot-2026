package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public interface ClimberSubsystem extends Subsystem, LoggableInputs {


    @AutoLog
    public class ClimberInputs {
        public LinearVelocity newVelocity = InchesPerSecond.zero();
        public LinearVelocity velocity = InchesPerSecond.zero();
        public LinearVelocity lastVelocity = InchesPerSecond.zero();
        public Distance height = Inches.zero();
        public boolean canGoUp, canGoDown;

    }

    ClimberInputsAutoLogged getSensors();

    default void stop() {
    }

    default void setVelocity(LinearVelocity newVelocity) {
    }

    default LinearVelocity getVelocity() {
        return this.getSensors().velocity;
    }

    default Distance getHeight() {
        return this.getSensors().height;
    }

}
