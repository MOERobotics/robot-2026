package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ClimberSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.InchesPerSecond;

public class ClimberTestCommand extends Command {
    public ClimberSubsystem climber;
    public Joystick joyStick;
    public int clicksSequence;
    public int ticks;
    public boolean decayTrigger;

    public ClimberTestCommand(RobotContainer robot, Joystick joystick){
        this.climber = robot.getClimber();
        this.joyStick = joystick;
        this.clicksSequence = 0;
        this.ticks = 0;
        this.decayTrigger = false;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        if (joyStick.getRawButton(1)) {
            climber.setVelocity(InchesPerSecond.of(.8));
        } else if (joyStick.getRawButton(2)) {
            climber.setVelocity(InchesPerSecond.of(-.8));
        } else {
            climber.stop();
        }
        if (joyStick.getRawButtonPressed(3)){
            clicksSequence += 1;
            decayTrigger = true;
        }
        if (decayTrigger){
            clicksSequenceDecay();
        }
        Logger.recordOutput("clicks",clicksSequence);
        Logger.recordOutput("ticks", ticks);

        if (clicksSequence == 3){
            climber.unlatchHooks();
        }

    }

    public void clicksSequenceDecay(){
        ticks += 1;
        if (ticks > 100){
            clicksSequence = 0;
            ticks = 0;
            decayTrigger = false;
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

