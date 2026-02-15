package frc.robot.subsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

public interface ShooterSubsystem extends Subsystem, LoggableInputs {

    @AutoLog
    class ShooterInputs{
        public Angle turretAngle = Degrees.zero();

        public Angle hoodAngle = Degrees.zero();

        public boolean shooting = false;

        public boolean spindexerOn = false;

        public boolean transitionOn = false;

        public LinearVelocity flywheelSpeed = MetersPerSecond.zero();

        public LinearVelocity spinDexerSpeed =  MetersPerSecond.zero();
        public LinearVelocity transitionSpeed =  MetersPerSecond.zero();


        public LinearVelocity turretSpeed = MetersPerSecond.zero();

        public LinearVelocity hoodSpeed = MetersPerSecond.zero();

        public Angle turretTargetAngle = Degrees.zero();

        public Angle hoodTargetAngle = Degrees.zero();



    }

    public ShooterInputs getSensors();


    void setTurretTarget(Angle angle);

    void setHoodTarget(Angle angle);

    void setFlywheelTargetRPM(double rpm);

    void loadFuel(double spindexerPower, double transitionPower);

    void stopFeeding();

    boolean isTurretAtTarget();

    boolean isHoodAtTarget();

    boolean isFlywheelAtSpeed();

    boolean isReadyToShoot();


    default Angle getTurretAngleinDegrees() {
        return getSensors().turretAngle;
    }

    default Angle getHoodAngleinDegrees() {
        return getSensors().hoodAngle;
    }

    default LinearVelocity getFlywheelSpeed() {
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

    default Angle getTurretTargetAngle() {
        return getSensors().turretTargetAngle;
    }

    default Angle getHoodTargetAngle() {
        return getSensors().hoodTargetAngle;
    }






}
