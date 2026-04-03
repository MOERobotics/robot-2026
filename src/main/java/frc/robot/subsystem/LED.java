package frc.robot.subsystem;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.interfaces.LEDSubsystem;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class LED implements LEDSubsystem {
    AddressableLED moeLED;
    AddressableLEDBuffer moeLEDBuffer;

    public LED(int ledBufferLength, int portNumber){
        moeLEDBuffer = new AddressableLEDBuffer(ledBufferLength);
        moeLED = new AddressableLED(portNumber);
        moeLED.setLength(ledBufferLength);
        moeLED.setData(moeLEDBuffer);
        moeLED.start();

    }

    private void setPattern(LEDPattern pattern) {
        pattern.applyTo(moeLEDBuffer);
        moeLED.setData(moeLEDBuffer);


    }
    public void setPatternBlinking(Color color){
        LEDPattern blinking = LEDPattern.solid(color);
        blinking.blink(Seconds.of(1));
        setPattern(blinking);
    }
    public void setSolidColor(Color color){
        LEDPattern solidColor = LEDPattern.solid(color);
        setPattern(solidColor);
    }

    @Override
    public void toLog(LogTable logTable) {

    }

    @Override
    public void fromLog(LogTable logTable) {

    }
}