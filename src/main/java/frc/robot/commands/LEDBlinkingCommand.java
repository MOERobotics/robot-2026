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
import org.littletonrobotics.junction.Logger;

import java.awt.*;

import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kLimeGreen;
import static java.awt.Color.red;

public class LEDBlinkingCommand extends Command {
    public LEDSubsystem LED;
    Color colorForBlinking;



    public LEDBlinkingCommand(Color colorForBlinking, RobotContainer robot) {
        this.LED = robot.getLed();
       this.colorForBlinking = colorForBlinking;
       addRequirements(robot.led);

       }


    public void execute() {
        LED.setPatternBlinking(colorForBlinking);
        Logger.recordOutput("LEDBlinking", true);
        Logger.recordOutput("LEDBlinkingColor", colorForBlinking);
    }

    @Override
    public void end(boolean interrupted) {
        LED.setSolidColor(kBlack);
        Logger.recordOutput("LEDBlinking", false);


    }
}

