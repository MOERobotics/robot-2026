package frc.robot.subsystem.fakes;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;
import frc.robot.subsystem.interfaces.ClimberSubsystem;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberFake extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberSubsystem {


    public ClimberFake() {
        super(new ClimberInputsAutoLogged());

    }

    @Override
    public ClimberInputsAutoLogged getSensors() {
        return null;
    }

    @Override
    public void stop() {
        ClimberSubsystem.super.stop();
    }

    @Override
    public void setVelocity(LinearVelocity newVelocity) {
        ClimberSubsystem.super.setVelocity(newVelocity);
    }

    @Override
    public LinearVelocity getVelocity() {
        return ClimberSubsystem.super.getVelocity();
    }

    @Override
    public Distance getHeight() {
        return ClimberSubsystem.super.getHeight();
    }

    @Override
    public void unlatchHooks() {
        ClimberSubsystem.super.unlatchHooks();
    }
}
