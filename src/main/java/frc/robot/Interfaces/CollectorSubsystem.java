package frc.robot.Interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface CollectorSubsystem extends Subsystem {

    @AutoLog
    public class CollectorInputs {
        public AngularVelocity wheelVelocity = RPM.zero();
        public AngularVelocity collectorArmVelocity = RadiansPerSecond.zero();
        public boolean inStartPosition = false;
        public boolean inCollectPosition = false;
        public Angle collectorArmAngle = Degrees.zero();
    }
   CollectorInputs getSensors();
    public void setArmVelocity(AngularVelocity armVelocity);
    public void setWheelVelocity(AngularVelocity wheelVelocity);
    public Angle getArmAngle();
    public boolean inStartPosition() ;
    public boolean inCollectPosition() ;
}
