package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.LED;
import frc.robot.subsystem.interfaces.LEDSubsystem;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;

import java.awt.*;

import static edu.wpi.first.wpilibj.util.Color.kLimeGreen;
import static java.awt.Color.red;

public class LEDBlinkingCommand extends Command {
    public LEDSubsystem LED;
    AddressableLED moeLED;
    AddressableLEDBuffer moeLEDBuffer;
    public int timePassed = 0;


    public LEDBlinkingCommand(RobotContainer robot, AddressableLED moeLED, AddressableLEDBuffer moeLEDBuffer) {
        this.LED = robot.;
        this.moeLED = moeLED;
       this.moeLEDBuffer = moeLEDBuffer;

       }


    public void execute(Color colorForBlinking) {
        colorForBlinking = kLimeGreen ;
        LED.setPatternBlinking(colorForBlinking);
    }

    @Override
    public void end(boolean interrupted) {
    }
}

