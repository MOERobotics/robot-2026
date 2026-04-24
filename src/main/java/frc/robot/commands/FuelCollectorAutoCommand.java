package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class FuelCollectorAutoCommand extends Command {
    CollectorSubsystem collectorSubsystem;
    boolean shouldGoToStartPosition;
    boolean shouldGoToCollectPosition;
    double armkp = 5.0 /600;
    double armki = 0.02/180;
    double armkd = 0.0002/180;
    double feedforward;
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
        double rollerPower = 0;
        Angle targetArmPosition;
        Angle currentArmPosition;

        if (collectRollers.equals("in")) {
            rollerPower = -1;
        } else if (collectRollers.equals("out")) {
            rollerPower = 1;
        } else if (collectRollers.equals("stop")) {
            rollerPower = 0;

        }
        collectorSubsystem.setRollerPower(rollerPower);
        currentArmPosition = collectorSubsystem.getArmAngle();

        if (collectPosition) {
            targetArmPosition = Degrees.of(135);
        } else {
            targetArmPosition = Degrees.of(215);
        }

        fuelCollectorArmPID.setSetpoint(targetArmPosition.in(Degrees));


        double armCorrection = MathUtil.clamp(fuelCollectorArmPID.calculate(currentArmPosition.in(Degrees)), -0.3, 0.6);
        if(!fuelCollectorArmPID.atSetpoint()){
            collectorSubsystem.setArmVelocity(RPM.of(armCorrection));
        } else {
            targetArmPosition = Degrees.of(currentArmPosition.in(Degrees));
            collectorSubsystem.setArmVelocity(RPM.of(0));
        }
            Logger.recordOutput("FuelCollector/ArmCorrection", armCorrection);
            Logger.recordOutput("FuelCollector/ArmCorrectionVel", DegreesPerSecond.of(armCorrection));

        Logger.recordOutput("at setpoint", fuelCollectorArmPID.atSetpoint());
      //  Logger.recordOutput("actual collecotr velocity", collectorSubsystem.getArmVelocity());
        Logger.recordOutput("TargetArmPos", targetArmPosition);


    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.setArmVelocity(RPM.zero());
        collectorSubsystem.setRollerPower(0);
    }

    @Override
    public boolean isFinished() {
        if(hold){
            return false;
        }
        return fuelCollectorArmPID.atSetpoint();
    }


}
