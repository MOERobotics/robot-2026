package frc.robot.subsystem.fakes;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ShooterInputsAutoLogged;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import org.littletonrobotics.junction.LogTable;

public class ShooterFake extends MOESubsystem<ShooterInputsAutoLogged> implements ShooterSubsystem {

    public ShooterFake() {
        super(new ShooterInputsAutoLogged());
    }

    @Override
    public void setTurretPower(double power) {

    }

    @Override
    public void setHoodPower(double power) {

    }

    @Override
    public void setFlywheelPower(double power) {

    }

    @Override
    public void loadFuel(double spindexerPower, double transitionPower) {
        ShooterSubsystem.super.loadFuel(spindexerPower, transitionPower);
    }

    @Override
    public void setTransitionPower(double transitionPower) {

    }

    @Override
    public void setSpindexerPower(double spindexerPower) {

    }

    @Override
    public void stopFeeding() {

    }

    @Override
    public boolean reachedHoodMax() {
        return ShooterSubsystem.super.reachedHoodMax();
    }

    @Override
    public boolean reachedHoodMin() {
        return ShooterSubsystem.super.reachedHoodMin();
    }

    @Override
    public boolean reachedTurretMax() {
        return ShooterSubsystem.super.reachedTurretMax();
    }

    @Override
    public boolean reachedTurretMin() {
        return ShooterSubsystem.super.reachedTurretMin();
    }

    @Override
    public Angle getTurretAngleinDegrees() {
        return ShooterSubsystem.super.getTurretAngleinDegrees();
    }
    @Override
    public Angle getHoodAngleFromMotor() {
        return ShooterSubsystem.super.getHoodAngleFromMotor();
    }
    @Override

    public AngularVelocity getFlywheelSpeed() {
        return ShooterSubsystem.super.getFlywheelSpeed();
    }

    @Override
    public boolean getSpindexerOn() {
        return ShooterSubsystem.super.getSpindexerOn();
    }

    @Override
    public boolean getTransitionOn() {
        return ShooterSubsystem.super.getTransitionOn();
    }

    @Override
    public void toLog(LogTable logTable) {

    }

    @Override
    public void fromLog(LogTable logTable) {

    }
}
