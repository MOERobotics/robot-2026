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
        public LinearVelocity velocity = InchesPerSecond.zero();
        public Distance height = Inches.zero();
        public boolean canGoUp, canGoDown;
        public boolean hooksLatched = true;
        public double appliedPower = 0;

    }

    ClimberInputsAutoLogged getSensors();

    default void stop() {
    }

    default void setPower(double power) {
    }

    default LinearVelocity getVelocity() {
        return this.getSensors().velocity;
    }

    default Distance getHeight() {
        return this.getSensors().height;
    }

    default void unlatchHooks(){
    }

}
