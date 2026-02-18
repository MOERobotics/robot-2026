package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.Interfaces.CollectorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class FuelCollectorTeleopCommand extends Command {

    private final CollectorSubsystem fuelCollector;
    private final Joystick joystick;

    private AngularVelocity armVelocity;
    private AngularVelocity rollerVelocity;

    public FuelCollectorTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.fuelCollector = robot.getFuelCollector();
        this.joystick = joystick;

        addRequirements(fuelCollector);
    }

    @Override
    public void initialize() {
        armVelocity = RPM.zero();
        rollerVelocity = RPM.zero();
    }

    @Override
    public void execute() {

        armVelocity = RPM.of(
                -0.5 * MathUtil.applyDeadband(joystick.getRawAxis(0), 0.05)
        );

        if (joystick.getRawButton(0)) {
            rollerVelocity = RPM.of(1);
        } else if (joystick.getRawButton(1)) {
            rollerVelocity = RPM.of(-1);
        } else {
            rollerVelocity = RPM.zero();
        }

        fuelCollector.setArmVelocity(armVelocity);
        fuelCollector.setRollerVelocity(rollerVelocity);

        Logger.recordOutput("FuelCollector/ArmVelocity", armVelocity);
        Logger.recordOutput("FuelCollector/RollerVelocity", rollerVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        fuelCollector.setArmVelocity(RPM.zero());
        fuelCollector.setRollerVelocity(RPM.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}