package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

public interface MOESimulator {
    static AngularVelocity decelerate(AngularVelocity velocity, double decelerationCoef) {
        if (velocity.isNear(RPM.zero(), 0.01)) {
            return RPM.zero();
        } else if (velocity.abs(RPM) > decelerationCoef) {
            return velocity.minus(RPM.of(Math.copySign(decelerationCoef, velocity.in(RPM))));
        } else {
            return RPM.zero();
        }
    }

    static AngularVelocity getSystemVelocity(AngularVelocity motorVelocity, double gearing) {
        return motorVelocity.div(gearing);
    }

    static AngularVelocity getMotorVelocity(AngularVelocity systemVelocity, double gearing) {
        return systemVelocity.times(gearing);
    }

    public void updateSimState();
}
