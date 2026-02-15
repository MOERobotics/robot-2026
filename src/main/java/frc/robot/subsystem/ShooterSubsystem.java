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

        public boolean transitonOn = false;

        public LinearVelocity flywheelSpeed = MetersPerSecond.zero();


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

    default boolean getSpindexerOn() {
        return getSensors().spindexerOn;
    }

    default boolean getTransitionOn() {
        return getSensors().transitonOn;
    }

    default boolean isLoggedReadyToShoot() {
        return this.isReadyToShoot();
    }


}
