package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;

public class FuelCollectorTestCommand extends Command {
    CollectorSubsystem collectorSubsystem;

    Joystick joystick;

    // private AngularVelocity armVelocity;
    // private AngularVelocity rollerVelocity;

    public FuelCollectorTestCommand(RobotContainer robot, Joystick joystick) {
        this.collectorSubsystem = robot.getCollector();
        this.joystick = joystick;
        addRequirements(collectorSubsystem);

    }

    @Override
    public void initialize() {
        //armVelocity = RPM.zero();
        //  rollerVelocity = RPM.zero();

    }


    @Override
    public void execute() {

        AngularVelocity armVelocity = RPM.of(-0.5 * MathUtil.applyDeadband(joystick.getRawAxis(0), 0.05));
        AngularVelocity rollerVelocity;
        if (joystick.getRawButton(1)) {
            rollerVelocity = RPM.of(1);
        } else if (joystick.getRawButton(2)) {
            rollerVelocity = RPM.of(-1);
        } else {
            rollerVelocity = RPM.zero();
        }

        collectorSubsystem.setArmVelocity(armVelocity);
        collectorSubsystem.setRollerVelocity(rollerVelocity);




    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.setArmVelocity(RPM.zero());
        collectorSubsystem.setRollerVelocity(RPM.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}