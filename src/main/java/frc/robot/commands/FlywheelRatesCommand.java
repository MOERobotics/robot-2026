package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
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
        double sumOfRPM = 0;
        for (double __rpm : _rpm) {
            sumOfRPM+=__rpm;
        }

        Logger.recordOutput(("Flywheel RPM at " + power), sumOfRPM / _rpm.length);

        shooterSubsystem.setFlywheelPower(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(6);
    }
}
