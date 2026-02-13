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

    void setVelocity();

    @AutoLog
    static class ClimberInputs{
        double newVelocity = 0;
        LinearVelocity velocity = InchesPerSecond.zero();
        Distance height = Inches.zero();
        boolean canGoUp, canGoDown;
    }
    void setVelocity(double newVelocity);
    void getVelocity();
    Distance getHeight();
    boolean checkCanGoUp();
    boolean checkCanGoDown();

}
