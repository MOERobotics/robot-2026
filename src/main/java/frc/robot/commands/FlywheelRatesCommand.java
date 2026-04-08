package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RPM;

public class FlywheelRatesCommand extends Command {
    ShooterSubsystem shooterSubsystem;


    double power;
    Timer timer = new Timer();

    int i = 0;
    double[] _rpm = new double[50];

    public double finalRPM =0;




    public FlywheelRatesCommand(RobotContainer robot, double power){
        this.shooterSubsystem = robot.getShooterSubsystem();
        this.power = power;
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
        timer.start();
    }

    @Override
    public void execute() {
        shooterSubsystem.setFlywheelPower(power);
        i = (i + 1) % 50;
        _rpm[i] = shooterSubsystem.getFlywheelSpeed().in(RPM);

    }



    @Override
    public void end(boolean interrupted) {

        Logger.recordOutput(("Flywheel RPM at " + power),finalRPM);
        shooterSubsystem.setFlywheelPower(0);
    }

    public double getFinalRPM (){

        return finalRPM;

    };
    @Override
    public boolean isFinished() {
        double sumOfRPM = 0;
        for (double __rpm : _rpm) {
            sumOfRPM+=__rpm;
        }

        finalRPM = sumOfRPM/_rpm.length;
        return timer.hasElapsed(4);
    }
}
