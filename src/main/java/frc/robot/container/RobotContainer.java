package frc.robot.container;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.TankDriveSubsystem;
import frc.robot.subsystem.interfaces.ClimberSubsystem;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract @Data class RobotContainer implements LoggableInputs {
    private TankDriveSubsystem tankDrive;
    SwerveDriveSubsystem robotSwerveDrive;
    ClimberSubsystem climber;
    private ShooterSubsystem shooterSubsystem;
    private SwerveDriveSubsystem robotSwerveDrive;
    private PowerDistribution pdh;

    public RobotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("Shooter", shooterSubsystem);
        table.put("SwerveDrive", robotSwerveDrive);
        table.put("Climber", climber);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
        robotSwerveDrive = table.get("SwerveDrive", robotSwerveDrive);
        climber = table.get("Climber", climber);
        shooterSubsystem = table.get("Shooter", shooterSubsystem);
    }
}
