package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.RPM;

public class FlywheelRatesCommand extends Command {
    ShooterSubsystem shooterSubsystem;
    double power;
    Timer timer = new Timer();


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
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(("FlywheelTimers/Max FlywheelRPM at: " + power), shooterSubsystem.getFlywheelSpeed().in(RPM));
        shooterSubsystem.setFlywheelPower(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(10);
    }
}
