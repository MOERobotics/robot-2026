package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class FuelCollectorTeleopCommand extends Command {
    CollectorSubsystem collectorSubsystem;
    boolean shouldGoToStartPosition;
    boolean shouldGoToCollectPosition;
    Joystick joystick;
    double armkp = 0.1/180;
    double armki = 0.02/180;
    double armkd = 0.0002/180;
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
        fuelCollectorArmPID.setTolerance(1);
        fuelCollectorArmPID.setSetpoint(collectorSubsystem.getArmAngle().in(Degrees));
    }


    @Override
    public void execute() {
        AngularVelocity rollerVelocity;
        Angle targetArmPosition;
        Angle currentArmPosition;
        int collectorINButton = 6;
        int collectorOUTButton = 5;
        if (joystick.getRawButton(collectorINButton)) {
            rollerVelocity = RPM.of(0.6);
        } else if (joystick.getRawButton(collectorOUTButton)) {
            rollerVelocity = RPM.of(-0.6);
        } else {
            rollerVelocity = RPM.of(0);

        }
        collectorSubsystem.setRollerVelocity(rollerVelocity);
        int collectorStartPositionButton = 7;
        int collectorIntakePositionButton = 8;
        currentArmPosition = collectorSubsystem.getArmAngle();

        if (joystick.getRawButtonPressed(collectorStartPositionButton)) {
            shouldGoToStartPosition = true;
            shouldGoToCollectPosition = false;


        } else if (joystick.getRawButtonPressed(collectorIntakePositionButton)) {
            shouldGoToCollectPosition = true;
            shouldGoToStartPosition = false;
        }


        if (shouldGoToStartPosition) {
            targetArmPosition = Degrees.of(215);


        } else if (shouldGoToCollectPosition) {
            targetArmPosition = Degrees.of(135);
        } else {
            targetArmPosition = currentArmPosition;
        }

        Logger.recordOutput("ShouldGoToCollectPosition", shouldGoToCollectPosition);
        Logger.recordOutput("shouldGoToStartPosition", shouldGoToStartPosition);
        fuelCollectorArmPID.setSetpoint(targetArmPosition.in(Degrees));

        if (!fuelCollectorArmPID.atSetpoint()) {
            double armCorrection = fuelCollectorArmPID.calculate(currentArmPosition.in(Degrees));
            collectorSubsystem.setArmVelocity(DegreesPerSecond.of(armCorrection));
            Logger.recordOutput("FuelCollector/ArmCorrection", DegreesPerSecond.of(armCorrection));
        } else {
            shouldGoToCollectPosition = false;
            shouldGoToStartPosition = false;
            collectorSubsystem.setArmVelocity(DegreesPerSecond.of(0));
        }

        Logger.recordOutput("TargetArmPos", targetArmPosition);
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