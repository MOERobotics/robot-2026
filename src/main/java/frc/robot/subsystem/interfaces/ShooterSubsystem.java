package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public interface ShooterSubsystem extends Subsystem, LoggableInputs {


    @AutoLog
    class ShooterInputs{
        public Angle turretAngle = Degrees.zero();

        public Angle hoodAngleThroughbore = Rotation.zero();

        public Angle hoodAngleMotor = Rotation.zero();

        public double turretAngleDegrees = 0;

        public double hoodAngleThroughboreDegrees = 0;

        public boolean shooting = false;

        public boolean spindexerOn = false;

        public boolean transitionOn = false;

        public AngularVelocity flywheelSpeed = RPM.zero();

        public AngularVelocity spindexerSpeed =  RPM.zero();
        public AngularVelocity transitionSpeed =  RPM.zero();

        public AngularVelocity turretSpeed = RPM.zero();

        public AngularVelocity hoodSpeed = RPM.zero();

        public double spindexerPower=  0;

        public double turretPower=  0;
        public double flywheelPower=  0;
        public double transitionPower=  0;
        public double hoodPower=  0;

        public double turretMaxAngle = 0;
        public double turretMinAngle = 0;
        public double hoodMaxAngle = 0;
        public double hoodMinAngle = 0;
        public boolean reachedMaxTurret = false;
        public boolean reachedMinTurret = false;
        public boolean reachedMaxHood = false;
        public boolean reachedMinHood = false;
        public boolean atShooterSpeed = false;

        public double transitionCurrent =0;

        public boolean transitionCurrentLimit =false;
    }

    public ShooterInputs getSensors();


    void setTurretPower(double power);

    void setHoodPower(double power);

    void setFlywheelPower(double power);

    default void loadFuel(double spindexerPower, double transitionPower){
        setTransitionPower(transitionPower);
        setSpindexerPower(spindexerPower);
    };

    void setTransitionPower(double transitionPower);

    void setSpindexerPower(double spindexerPower);


    void stopFeeding();

    default boolean reachedHoodMax(){
        return getSensors().reachedMaxHood;
    };

    default boolean reachedHoodMin(){
        return getSensors().reachedMinHood;
    };

    default boolean reachedTurretMax(){
        return getSensors().reachedMaxTurret;
    };

    default boolean reachedTurretMin(){
        return getSensors().reachedMinTurret;
    };




    default Angle getTurretAngle() {
        return getSensors().turretAngle;
    }

    //default Angle getTurretAngleInRotation() {
    //    return getSensors().turretRotation;
    //}

    default Angle getHoodAngleFromThroughbore() {
        return getSensors().hoodAngleThroughbore;
    }

    default Angle getHoodAngleFromMotor() {
        return getSensors().hoodAngleMotor;
    }

    default AngularVelocity getFlywheelSpeed() {
        return getSensors().flywheelSpeed;
    }

    default boolean getSpindexerOn() {
        return getSensors().spindexerOn;
    }

    default boolean getTransitionOn() {
        return getSensors().transitionOn;
    }



}
