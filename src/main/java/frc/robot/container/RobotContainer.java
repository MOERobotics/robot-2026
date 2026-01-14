package frc.robot.container;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.TankDriveSubsystem;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract @Data class RobotContainer implements LoggableInputs {
    private TankDriveSubsystem tankDrive;
    private PowerDistribution pdh;

    public RobotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
    }
}
