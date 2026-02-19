package frc.robot.container;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.subsystem.SwerveDriveSubsystem;
import frc.robot.subsystem.TankDriveSubsystem;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import simulators.ClimberSim;

public abstract @Data class RobotContainer implements LoggableInputs {
    private TankDriveSubsystem tankDrive;
    SwerveDriveSubsystem robotSwerveDrive;
    ClimberSubsystem climber;
    private PowerDistribution pdh;

    public RobotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("SwerveDrive", robotSwerveDrive);
        table.put("Climber", climber);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
        robotSwerveDrive = table.get("SwerveDrive", robotSwerveDrive);
        climber = table.get("Climber", climber);
    }
}
