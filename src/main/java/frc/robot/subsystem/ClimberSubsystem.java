package frc.robot.subsystem;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public interface ClimberSubsystem extends Subsystem, LoggableInputs {



    @AutoLog
    static class ClimberInputs{
        LinearVelocity newVelocity = InchesPerSecond.zero();
        double pidError = 0;
        LinearVelocity velocity = InchesPerSecond.zero();
        LinearVelocity lastVelocity = InchesPerSecond.zero();
        Distance height = Inches.zero();
        boolean canGoUp, canGoDown;

    }
    ClimberInputsAutoLogged getSensors();

    default void stopVelocity() {}
    default void setVelocity(LinearVelocity newVelocity) {}
    default LinearVelocity getVelocity() {return this.getSensors().velocity;}
    default Distance getHeight() {return this.getSensors().height;}
    /*
    default boolean checkCanGoUp() {return this.getSensors().canGoUp;}
    default boolean checkCanGoDown() {return this.getSensors().canGoDown;}

     */
}
