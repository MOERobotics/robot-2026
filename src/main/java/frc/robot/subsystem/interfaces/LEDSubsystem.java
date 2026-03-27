package frc.robot.subsystem.interfaces;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LEDSubsystem extends Subsystem, LoggableInputs {

    public void setPatternBlinking(Color color);
    public void setSolidColor(Color color);
}
