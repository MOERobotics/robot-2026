package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public interface TankDriveSubsystem extends Subsystem, LoggableInputs {
    @AutoLog
    class DriveInputs {
        public Distance leftPosition = Inches.zero();
        public Distance rightPosition = Inches.zero();
        public Angle angle = Degrees.zero();
        public Pose2d pose = Pose2d.kZero;
        public LoggableInputs debugInfo;
    }

    public DriveInputs getSensors();

    // Outputs
    public void drive(double leftPercent, double rightPercent);

    // Inputs
    public default Distance getLeftPosition() {
        return getSensors().leftPosition;
    }

    public default Distance getRightPosition() {
        return getSensors().rightPosition;
    }

    public default Angle getAngle() {
        return getSensors().angle;
    }


}
