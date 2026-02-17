package frc.robot.subsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public interface ShooterSubsystem extends Subsystem, LoggableInputs {

    @AutoLog
    class ShooterInputs{
        public Angle turretAngle = Degrees.zero();

        public Angle hoodAngle = Degrees.zero();

        public boolean shooting = false;

        public boolean spindexerOn = false;

        public boolean transitionOn = false;

        public AngularVelocity flywheelSpeed = RPM.zero();

        public LinearVelocity spinDexerSpeed =  MetersPerSecond.zero();
        public LinearVelocity transitionSpeed =  MetersPerSecond.zero();


        public LinearVelocity turretSpeed = MetersPerSecond.zero();

        public LinearVelocity hoodSpeed = MetersPerSecond.zero();

        public boolean reachedMaxTurret = false;
        public boolean reachedMinTurret = false;
        public boolean reachedMaxHood = false;
        public boolean reachedMinHood = false;



    }

    public ShooterInputs getSensors();


    void setTurretPower(double power);

    void setHoodPower(double power);

    void setFlywheelPower(double power);

    void loadFuel(double spindexerPower, double transitionPower);

    void setTransitionPower(double transitionPower);

    void setSpindexerPower(double spindexerPower);


    void stopFeeding();

    boolean isFlywheelAtSpeed();

    boolean isReadyToShoot();

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




    default Angle getTurretAngleinDegrees() {
        return getSensors().turretAngle;
    }

    default Angle getHoodAngleinDegrees() {
        return getSensors().hoodAngle;
    }

    default AngularVelocity getFlywheelSpeed() {
        return getSensors().flywheelSpeed;
    }

    default LinearVelocity getSpindexerSpeed() {
        return getSensors().spinDexerSpeed;
    }
    default LinearVelocity getTransitionSpeed() {
        return getSensors().transitionSpeed;
    }
    default LinearVelocity getTurretSpeed() {
        return getSensors().turretSpeed;
    }

    default LinearVelocity getHoodSpeed() {
        return getSensors().hoodSpeed;
    }


    default boolean getSpindexerOn() {
        return getSensors().spindexerOn;
    }

    default boolean getTransitionOn() {
        return getSensors().transitionOn;
    }

    default boolean isLoggedReadyToShoot() {
        return this.isReadyToShoot();
    }








}
