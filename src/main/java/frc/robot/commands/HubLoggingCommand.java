package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

public class HubLoggingCommand extends Command {

    public Joystick driverJoystick;
    public HubLoggingCommand (Joystick joystick){
        this.driverJoystick = joystick;

    }
    @Override
    public void execute() {

        // logs match time
        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Time", matchTime);

        // gets alliance
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        String allianceStr = " ";

        if (allianceOpt.isPresent()) {
            if (allianceOpt.get() == DriverStation.Alliance.Red) {
                allianceStr = "Red";
            } else if (allianceOpt.get() == DriverStation.Alliance.Blue) {
                allianceStr = "Blue";
            }
        }

        SmartDashboard.putString("Match Alliance", allianceStr);

        // gets data on first inactive alliance (B or R)

        String gameData = DriverStation.getGameSpecificMessage();
        char autoLoser =  ' ';
        if(!gameData.isEmpty()){
            autoLoser = gameData.charAt(0);
        }

        // if red loses auto then they are inactive first and vice versa
        if (autoLoser == 'R') {
            SmartDashboard.putString("First Inactive", "Red");
        } else if (autoLoser == 'B')
            SmartDashboard.putString("First Inactive", "Blue");

        else {
            SmartDashboard.putString("First Inactive", "None");
        }


        int shift = 0;

        if (matchTime <= 130 && matchTime > 105) {
            shift = 1;

        } else if(matchTime <= 105 && matchTime > 80){
            shift = 2;

        } else if(matchTime <=80 &&matchTime >55){
            shift =3;
        }else if (matchTime <= 55 && matchTime > 30) {
            shift = 4;
        }
        /*
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);


        if (matchTime <= 135 && matchTime > 130) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if(matchTime <= 110 && matchTime > 105){
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if(matchTime <=85 &&matchTime >80){
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }else if (matchTime <= 60 && matchTime > 55) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }


         */
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);


        if (matchTime <= 135 && matchTime > 134.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if(matchTime <= 134.625 && matchTime > 134.375){
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else if(matchTime <=134.25 &&matchTime >134.125){
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }else if (matchTime <= 134 && matchTime > 133.75) {
            driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        }   else if(matchTime <= 133.625 && matchTime > 133.375){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else if(matchTime <=133.25 &&matchTime >133.125){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    }else if (matchTime <= 133 && matchTime > 132.75) {
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else if(matchTime <= 132.625 && matchTime > 132.375){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else if(matchTime <=132.25 &&matchTime >132.125){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    }else if (matchTime <= 132 && matchTime > 131.75) {
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    }  else if(matchTime <= 131.625 && matchTime > 131.375){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else if(matchTime <=131.25 &&matchTime >131.125){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    }else if (matchTime <= 131 && matchTime > 130.75) {
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    }   else if(matchTime <= 130.625 && matchTime > 130.375){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else if(matchTime <=130.25 &&matchTime >130.125){
        driverJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    }



        SmartDashboard.putNumber("Current Shift", shift);


        boolean redHubActive = true;
        boolean blueHubActive = true;

        if (shift != 0 && (autoLoser == 'R' || autoLoser == 'B')) {

            boolean redActiveThisShift;

            if (shift == 2 || shift == 4){
                redActiveThisShift=true;
            }else{
                redActiveThisShift=false;
            }


            if (autoLoser == 'R') {
                redHubActive = redActiveThisShift;
                blueHubActive = !redActiveThisShift;
            } else {

                blueHubActive = redActiveThisShift;
                redHubActive = !redActiveThisShift;

            }

        }


        SmartDashboard.putBoolean("Red Hub Active", redHubActive);
        SmartDashboard.putBoolean("Blue Hub Active", blueHubActive);

        boolean ourHubActive = false;

        if (allianceOpt.isPresent()) {
            if (allianceOpt.get() == DriverStation.Alliance.Red && redHubActive) {

                ourHubActive = true;

            } else if (allianceOpt.get() == DriverStation.Alliance.Blue && blueHubActive) {

                ourHubActive = true;
            }
        }

        SmartDashboard.putBoolean("Our Hub Active", ourHubActive);


    }


}