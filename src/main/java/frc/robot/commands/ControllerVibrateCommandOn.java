package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ControllerVibrateCommandOn extends Command {

private final Joystick functionJoystick;
public Joystick driverJoystick;
public int timePassed = 0;

public ControllerVibrateCommandOn(Joystick joystick, Joystick functionJoystick) {
    this.driverJoystick = joystick;
    this.functionJoystick = functionJoystick;


}
public void controllersVibratingTrue(){
    driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble,1 );
    functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,1 );
    Logger.recordOutput("BUZZING", true);

}
public void controllersVibratingFalse(){
    driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble,0 );
    functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,0 );
    Logger.recordOutput("BUZZING", false);

}

@Override
public void execute() {
    timePassed = ++timePassed % 64;
    if (timePassed <=16){
        controllersVibratingTrue();
    }  else if (timePassed <=32) {
        controllersVibratingFalse();
    } else if (timePassed <=48){
        controllersVibratingTrue();
    } else {
        controllersVibratingFalse();
    }


}

@Override
public void end(boolean interrupted) {
    controllersVibratingFalse();
}
}

