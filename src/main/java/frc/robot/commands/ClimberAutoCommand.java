package frc.robot.commands;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ClimberSubsystem;

import static edu.wpi.first.units.Units.InchesPerSecond;

public class ClimberAutoCommand extends Command {
    public ClimberSubsystem climber;
    public boolean direction;
    public double power;
    public boolean hold;

    public ClimberAutoCommand(RobotContainer robot, boolean direction, double power, boolean hold){
        this.climber = robot.getClimber();
        this.direction = direction;
        this.power = power;
        this.hold = hold;
        addRequirements(climber);

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        if (direction) {
            climber.setPower(power);
        } else {
            climber.setPower(-power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        if (hold){
            return false;
        }
        if (direction){
            return !climber.getSensors().canGoUp;
        }
        else {
            return !climber.getSensors().canGoDown;
        }

    }
}

