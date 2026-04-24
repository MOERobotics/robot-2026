package frc.robot.commands;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.LEDSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.wpilibj.util.Color.*;

public class LEDVibrateCommand extends Command {
    boolean teamAllianceWonR = DriverStation.getGameSpecificMessage().equals("R") && DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
    boolean teamAllianceWonB = DriverStation.getGameSpecificMessage().equals("B") && DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
    boolean teamAllianceWon = teamAllianceWonR || teamAllianceWonB;
    public Joystick functionJoystick;
    public Joystick driverJoystick;
    public int timePassed = 0;
    public int timePassedBlinking = 0;
    public LEDSubsystem LED;


    public LEDVibrateCommand(Joystick joystick, Joystick functionJoystick, LEDSubsystem LED) {
        this.driverJoystick = joystick;
        this.functionJoystick = functionJoystick;
        this.LED = LED;
        addRequirements(LED);
    }

    public void controllersVibratingTrue() {
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 1);
        functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 1);
        Logger.recordOutput("BUZZING", true);

    }

    public void controllersVibratingFalse() {
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        Logger.recordOutput("BUZZING", false);
    }

    public void inactiveTransition() {
        // active to inactive

        controllerVibrateCommandOff();
        lEDBlinkingCommand(kRed);

    }

    public void activeTransition() {
        // inactive to active
        controllerVibrateCommandOn();
        lEDBlinkingCommand(kRed);
    }

    public void inactivePeriod() {

        controllersVibratingFalse();
        lEDColorCommand(kRed);
    }

    public void activePeriod() {

        controllersVibratingFalse();
        lEDColorCommand(kGreen);
    }

    public void controllerVibrateCommandOn() {
        timePassed = ++timePassed % 64;
        if (timePassed <= 16) {
            controllersVibratingTrue();
        } else if (timePassed <= 32) {
            controllersVibratingFalse();
        } else if (timePassed <= 48) {
            controllersVibratingTrue();
        } else {
            controllersVibratingFalse();
        }

    }

    public void controllerVibrateCommandOff() {
        timePassed = ++timePassed % 64;
        if (timePassed <= 16) {
            controllersVibratingTrue();
        } else if (timePassed <= 24) {
            controllersVibratingFalse();
        } else if (timePassed <= 40) {
            controllersVibratingTrue();
        } else if (timePassed <= 48) {
            controllersVibratingFalse();
        } else if (timePassed <= 56) {
            controllersVibratingTrue();
        } else {
            controllersVibratingFalse();
        }

    }

    public void lEDBlinkingCommand(Color colorForBlinking) {
        timePassedBlinking = ++timePassedBlinking % 32;
        if (timePassedBlinking <= 16) {
            LED.setSolidColor(colorForBlinking);
        } else {
            LED.setSolidColor(kBlack);
        }
        Logger.recordOutput("LEDBLINKING", true);

    }

    public void lEDColorCommand(Color color) {
        LED.setSolidColor(color);
        Logger.recordOutput("LEDCOLOR", color);
        Logger.recordOutput("LEDBLINKING", false);


    }


    @Override
    public void execute() {
        if (!teamAllianceWon) {
            if (DriverStation.getMatchTime() <= 140/*2:20*/ && DriverStation.getMatchTime() > 110/*1:50*/) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 110/*1:50*/ && DriverStation.getMatchTime() >= 105/*1:45*/) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 105/*1:45*/ && DriverStation.getMatchTime() > 85/*1:25*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 85/*1:25*/ && DriverStation.getMatchTime() >= 80/*1:20*/) {
                activeTransition();
            } else if (DriverStation.getMatchTime() < 80/*1:20*/ && DriverStation.getMatchTime() > 60/*1:00*/) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 60/*1:00*/ && DriverStation.getMatchTime() >= 55)/*55*/ {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 55/*55*/ && DriverStation.getMatchTime() > 35/*35*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 35 && DriverStation.getMatchTime() >= 30) {
                activeTransition();
            } else {
                activePeriod();
            }
        }
        if (teamAllianceWon) {
            if (DriverStation.getMatchTime() <= 140/*2:20*/ && DriverStation.getMatchTime() > 135)/*2:15*/ {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 135/*2:15*/ && DriverStation.getMatchTime() >= 130/*2:10*/) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 130/*2:10*/ && DriverStation.getMatchTime() > 110/*1:50*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 110/*1:50*/ && DriverStation.getMatchTime() >= 105/*1:45*/) {
                activeTransition();
            } else if (DriverStation.getMatchTime() < 105/*1:45*/ && DriverStation.getMatchTime() > 85/*1:25*/) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 85/*1:25*/ && DriverStation.getMatchTime() >= 80/*1:20*/) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 80/*1:20*/ && DriverStation.getMatchTime() > 60/*1:00*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 60 && DriverStation.getMatchTime() >= 55) {
                activeTransition();
            } else {
                activePeriod();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        controllersVibratingFalse();
        lEDColorCommand(kBlack);
    }
}
