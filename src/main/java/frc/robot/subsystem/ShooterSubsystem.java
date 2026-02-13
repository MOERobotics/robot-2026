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


    public void setSpindexer(double power);

    public void setTransition(double power);

    public void setFlywheelSpeed(double power);


    public default Angle getHoodAngleinDegrees(){ return getSensors().hoodAngle; }

    public default Angle getTurretAngleinDegrees(){ return getSensors().turretAngle; }

    public default boolean getShooting(){ return getSensors().shooting; }


    public default boolean getSpindexerOn(){ return getSensors().spindexerOn; }
    public default boolean getTransitionOn(){ return getSensors().transitonOn; }

    public default LinearVelocity getFlywheelSpeed(){ return getSensors().flywheelSpeed; }






}
