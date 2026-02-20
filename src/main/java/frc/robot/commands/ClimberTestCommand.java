package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.interfaces.ClimberSubsystem;

import java.awt.*;

import static edu.wpi.first.units.Units.InchesPerSecond;

public class ClimberTestCommand extends Command {
    public ClimberSubsystem climber;
    public Joystick joyStick;

    public ClimberTestCommand(RobotContainer robot, Joystick joystick){
        this.climber = robot.getClimber();
        this.joyStick = joystick;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        if (joyStick.getRawButton(1)) {
            climber.setVelocity(InchesPerSecond.of(1));
        } else if (joyStick.getRawButton(2)) {
            climber.setVelocity(InchesPerSecond.of(-1));
        } else {
            climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

