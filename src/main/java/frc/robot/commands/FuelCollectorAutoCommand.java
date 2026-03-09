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

public class FuelCollectorAutoCommand extends Command {
    CollectorSubsystem collectorSubsystem;
    boolean shouldGoToStartPosition;
    boolean shouldGoToCollectPosition;
    double armkp = 0.05/180;
    double armki = 0.02/180;
    double armkd = 0.0002/180;
    PIDController fuelCollectorArmPID = new PIDController(
            armkp, armki, armkd
    );
    boolean hold, collectPosition;
    String collectRollers;

    public FuelCollectorAutoCommand(RobotContainer robot, boolean hold, boolean collectPosition, String collectRollers) {
        this.collectorSubsystem = robot.getCollector();
        this.hold = hold;
        this.collectPosition = collectPosition;
        this.collectRollers = collectRollers;

        addRequirements(collectorSubsystem);
    }

    @Override
    public void initialize() {
        fuelCollectorArmPID.setTolerance(1);
        fuelCollectorArmPID.setSetpoint(collectorSubsystem.getArmAngle().in(Degrees));
    }


    @Override
    public void execute() {
        AngularVelocity rollerVelocity = RPM.of(0);
        Angle targetArmPosition;
        Angle currentArmPosition;

        if (collectRollers == "in") {
            rollerVelocity = RPM.of(1);
        } else if (collectRollers == "out") {
            rollerVelocity = RPM.of(-1);
        } else if (collectRollers == "stop") {
            rollerVelocity = RPM.of(0);

        }
        collectorSubsystem.setRollerVelocity(rollerVelocity);
        currentArmPosition = collectorSubsystem.getArmAngle();

        if (!collectPosition) {
            targetArmPosition = Degrees.of(215);
        } else {
            targetArmPosition = Degrees.of(135);
        }
        fuelCollectorArmPID.setSetpoint(targetArmPosition.in(Degrees));

            double armCorrection = fuelCollectorArmPID.calculate(currentArmPosition.in(Degrees));
            collectorSubsystem.setArmVelocity(RPM.of(armCorrection));
            Logger.recordOutput("FuelCollector/ArmCorrection", armCorrection);
            Logger.recordOutput("FuelCollector/ArmCorrectionVel", DegreesPerSecond.of(armCorrection));
        Logger.recordOutput("at setpoint", fuelCollectorArmPID.atSetpoint());
      //  Logger.recordOutput("actual collecotr velocity", collectorSubsystem.getArmVelocity());
        Logger.recordOutput("TargetArmPos", targetArmPosition);


    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.setArmVelocity(RPM.zero());
        collectorSubsystem.setRollerVelocity(RPM.zero());
    }

    @Override
    public boolean isFinished() {
        if(hold){
            return false;
        }
        return fuelCollectorArmPID.atSetpoint();
    }


}
