package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.Collector;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;

public class FuelCollectorTeleopCommand extends Command {
    CollectorSubsystem collectorSubsystem;

    Joystick joystick;
    double armkp =1;
    double armki = 1;
    double armkd=1;
    PIDController fuelCollectorArmPID = new PIDController(
            armkp, armki, armkd
    );


    public FuelCollectorTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.collectorSubsystem = robot.getCollector();
        this.joystick = joystick;
        addRequirements(collectorSubsystem);

    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {

        AngularVelocity rollerVelocity;
        AngularVelocity targetArmVelocity;
        AngularVelocity currentArmVelocity;
        int collectorINButton= 5;
        int collectorOUTButton= 6;
        if (joystick.getRawButtonPressed(collectorINButton)) {
             rollerVelocity = RPM.of(1);
        } else if (joystick.getRawButtonPressed(collectorOUTButton)) {
             rollerVelocity = RPM.of(-1);
        } else {  rollerVelocity = RPM.of(0);

    }
        collectorSubsystem.setRollerVelocity(rollerVelocity);
        int collectorStartButton = 7 ;
        int collectorIntakeButton = 8;

        if (joystick.getRawButton(collectorStartButton)) {
            currentArmVelocity = collectorSubsystem.getArmVelocity();
            targetArmVelocity = RPM.of(0.2);
            AngularVelocity armError = currentArmVelocity.minus(targetArmVelocity);
            double newVelocity = fuelCollectorArmPID.calculate(armError.in(RPM));
            collectorSubsystem.setArmVelocity(RPM.of(newVelocity));
        } else if (joystick.getRawButton(collectorIntakeButton)) {
            currentArmVelocity = collectorSubsystem.getArmVelocity();
            targetArmVelocity = RPM.of(-0.2);
            AngularVelocity armError = currentArmVelocity.minus(targetArmVelocity);
            double newVelocity = fuelCollectorArmPID.calculate(armError.in(RPM));
            collectorSubsystem.setArmVelocity(RPM.of(newVelocity));
        }

        //Logger.recordOutput("FuelCollector/ArmVelocity", armVelocity); TODO
        Logger.recordOutput("FuelCollector/RollerVelocity", rollerVelocity);
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
