package frc.robot.subsystem.fakes;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.CollectorInputsAutoLogged;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import org.littletonrobotics.junction.LogTable;

import static edu.wpi.first.units.Units.Degrees;

public class CollectorFake extends MOESubsystem<CollectorInputsAutoLogged> implements CollectorSubsystem {

    public CollectorFake() {
        super(new CollectorInputsAutoLogged());
        this.getSensors().collectorArmAngle = Degrees.zero();
    }

    @Override
    public void setArmVelocity(AngularVelocity armVelocity) {

    }

    @Override
    public void setRollerPower(double power) {

    }



    @Override
    public AngularVelocity getArmVelocity() {
        return null;
    }

    @Override
    public Angle getArmAngle() {
        return Degrees.zero();
    }

    @Override
    public boolean inStartPosition() {
        return false;
    }

    @Override
    public boolean inCollectPosition() {
        return false;
    }
}
