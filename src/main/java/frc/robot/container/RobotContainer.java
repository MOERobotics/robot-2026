package frc.robot.container;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.ClimberSubsystem;
import frc.robot.subsystem.interfaces.TankDriveSubsystem;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract @Data class RobotContainer implements LoggableInputs {
    private TankDriveSubsystem tankDrive;
    private ShooterSubsystem shooterSubsystem;
    private SwerveDriveSubsystem robotSwerveDrive;
    ClimberSubsystem climber;
    private PowerDistribution pdh;
    CollectorSubsystem fuelCollector;

    public RobotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("SwerveDrive", robotSwerveDrive);
        table.put ("FuelCollector", fuelCollector);

        table.put("Shooter", shooterSubsystem);
        table.put("Climber", climber);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
        robotSwerveDrive = table.get("SwerveDrive", robotSwerveDrive);
        shooterSubsystem = table.get("Shooter", shooterSubsystem);
        climber = table.get("Climber", climber);
        fuelCollector = table.get("FuelCollector", fuelCollector);
    }
}
