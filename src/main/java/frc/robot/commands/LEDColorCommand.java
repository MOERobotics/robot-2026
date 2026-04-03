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

public class LEDColorCommand extends Command {
    public LEDSubsystem LED;
    public Color solidColor;


    public LEDColorCommand(RobotContainer robot, Color solidColor) {
        this.LED = robot.getLed();
        this.solidColor = solidColor;
        addRequirements(robot.led);

    }


    public void execute() {
        LED.setSolidColor(solidColor);
        Logger.recordOutput("LEDSolidColorOn", true);
        Logger.recordOutput("LEDBlinkingColor", solidColor);

    }

    @Override
    public void end(boolean interrupted) {
        LED.setSolidColor(kBlack);
        Logger.recordOutput("LEDSolidColorOn", false);
    }
}
