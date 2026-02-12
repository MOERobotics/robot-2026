package frc.robot.Interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface CollectorSubsystem {
    @AutoLog
    class AlgaeCollectorInputs {
        public AngularVelocity wheelVelocity = RPM.zero();
        public Angle collectorArmAngle = Degrees.zero();
        public boolean inStartPosition;
        public boolean inCollectPosition;
        public AngularVelocity collectorArmVelocity = RadiansPerSecond.zero();

    }

    CollectorSubsystem getSensors();

    default void setArmVelocity(AngularVelocity ArmVelocity) {
    }

    default void setWheelVelocity(AngularVelocity wheelVelocity) {
    }

    default Angle getArmAngle() {
        return this.getSensors().getArmAngle();
    }

    default boolean inStartPosition() {
        return this.getSensors().inStartPosition();
    }

    default boolean inCollectPosition() {
        return this.getSensors().inCollectPosition();
    }

}