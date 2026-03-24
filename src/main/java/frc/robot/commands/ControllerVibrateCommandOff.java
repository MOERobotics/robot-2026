package frc.robot.commands;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ControllerVibrateCommandOff extends Command {

    private final Joystick functionJoystick;
    public Joystick driverJoystick;
    public int timePassed = 0;

    public ControllerVibrateCommandOff(Joystick joystick, Joystick functionJoystick) {
        this.driverJoystick = joystick;
        this.functionJoystick = functionJoystick;


    }
     public void controllersVibratingTrue(){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble,1);
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
        }  else if (timePassed <=24) {
            controllersVibratingFalse();
        } else if (timePassed <=40){
            controllersVibratingTrue();
        } else if (timePassed<=48 ) {
            controllersVibratingFalse();
        } else if (timePassed <=56) {
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

/*
        if (matchTime <= 135 && matchTime > 134.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 134.625 && matchTime > 134.375) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 134.25 && matchTime > 134.125) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 134 && matchTime > 133.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 133.625 && matchTime > 133.375) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 133.25 && matchTime > 133.125) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 133 && matchTime > 132.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 132.625 && matchTime > 132.375) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 132.25 && matchTime > 132.125) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 132 && matchTime > 131.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 131.625 && matchTime > 131.375) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 131.25 && matchTime > 131.125) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 131 && matchTime > 130.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 130.625 && matchTime > 130.375) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if (matchTime <= 130.25 && matchTime > 130.125) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }


        Logger.recordOutput("FlywheelOutput", output);


        boolean redHubActive = true;
        boolean blueHubActive = true;


    }

*/