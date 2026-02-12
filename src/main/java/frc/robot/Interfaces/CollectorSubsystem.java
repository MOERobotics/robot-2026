package frc.robot.Interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface CollectorSubsystem {
    class

    CollectorSubsystem getSensors();

    default void setArmVelocity(AngularVelocity armVelocity) {
    }

    default void setWheelVelocity(AngularVelocity wheelVelocity) {
    }


}
