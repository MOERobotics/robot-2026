package frc.robot.subsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class ShooterControl extends MOESubsystem <ShooterInputsAutoLogged> implements ShooterSubsystem{

    public SparkMax turretMotorControl;
    public SparkAbsoluteEncoder turretEncoder;

    public SparkAbsoluteEncoder hoodEncoder;
    public SparkMax hoodMotorControl;

    public SparkMax spindexerMotorControl;
    public SparkMax transitionMotorControl;

    public SparkMax flyWheelMotorControl;

    public static double HOOD_CONVERSION_FACTOR;

    public static double TURRET_CONVERSION_FACTOR;




    public ShooterControl(SparkMax turretMotorControl, SparkMax hoodMotorControl, SparkMax spindexerMotorControl, SparkMax transitionMotorControl, SparkMax flyWheelMotorControl, SparkAbsoluteEncoder turretEncoder, SparkAbsoluteEncoder hoodEncoder) {

        super(new ShooterInputsAutoLogged());

        this.flyWheelMotorControl = flyWheelMotorControl;
        this.turretMotorControl = turretMotorControl;
        this.hoodMotorControl = hoodMotorControl;
        this.spindexerMotorControl = spindexerMotorControl;
        this.transitionMotorControl = transitionMotorControl;
        this.turretEncoder = turretEncoder;
        this.hoodEncoder = hoodEncoder;

    }




    @Override
    public void setSpindexer(double power) {
        spindexerMotorControl.set(power);
    }

    @Override
    public void setTransition(double power) {
        transitionMotorControl.set(power);
    }

    @Override
    public void setFlywheelSpeed(double power) {
        flyWheelMotorControl.set(power);
    }

    @Override
    public Angle getHoodAngleinDegrees() {
        // TODO method to get encoder position to angle
        return Degrees.of(hoodEncoder.getPosition() * HOOD_CONVERSION_FACTOR);
    }

    @Override
    public Angle getTurretAngleinDegrees() {
        // TODO method to get encoder position to angle
        return Degrees.of((turretEncoder.getPosition()) * TURRET_CONVERSION_FACTOR);
    }

    @Override
    public boolean getShooting() {
        return  getTransitionOn() && getSpindexerOn();
    }

    @Override
    public boolean getSpindexerOn() {
        return Math.abs(spindexerMotorControl.get())>0;
    }

    @Override
    public boolean getTransitionOn() {
        return getSpindexerOn();
    }

    @Override
    public LinearVelocity getFlywheelSpeed() {
        return MetersPerSecond.of(flyWheelMotorControl.getAlternateEncoder().getVelocity());
    }
}
