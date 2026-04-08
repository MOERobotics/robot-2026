package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import java.util.Arrays;

public class CalibrationCommand extends Command {

    RobotContainer robot ;

    double[]power = {0.4,0.5,0.6};

     FlywheelRatesCommand [] flywheelRates = new FlywheelRatesCommand[power.length];


    double[] rpm = new double[power.length];

    int currentCount =0;
    private FlywheelRatesCommand currentCommand;


    public CalibrationCommand(RobotContainer robot){
        this.robot = robot;
    }


    public double m =0;

    public double b =0;
    @Override
    public void initialize() {

        for(int i =0; i < power.length; i++){
            flywheelRates[i] =  new FlywheelRatesCommand(robot, power[i]);
        }


        currentCommand = flywheelRates[0];
        currentCommand.schedule();
    }

    @Override
    public void execute() {

        if(currentCommand.isFinished() && currentCount < power.length){

            rpm[currentCount] = flywheelRates[currentCount].getFinalRPM();
            Logger.recordOutput("rpm" ,rpm[currentCount]);

            currentCount ++;
            if(currentCount < power.length){
                currentCommand = flywheelRates[currentCount];
                currentCommand.schedule();
            }

        }

    }

    @Override
    public void end(boolean interrupted) {
        double sumofX = 0;
        for (double __rpm : rpm) {
            sumofX+=__rpm;
        }

        double sumofY = 0;
        for (double __power : power) {
            sumofY+=__power;
        }


        double sumofBoth = 0;
        for (int i =0; i < power.length ; i++) {
            sumofBoth+= power[i]* rpm[i] ;
        }

        double sumofSqX = 0;
        for (double __rpm : rpm) {
            sumofSqX+=__rpm * __rpm;
        }


        m = ((power.length*sumofBoth) -(sumofY*sumofX)) / (power.length*sumofSqX - (sumofX*sumofX));


        b = (sumofY - m*sumofX)/power.length;

        Logger.recordOutput("m" ,1/m);

        Logger.recordOutput("b" ,b);


    }



    @Override
    public boolean isFinished() {
        return currentCount >= power.length;
    }
}
