package frc.robot.container;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.interfaces.*;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.pathplanner.lib.config.RobotConfig;
import org.photonvision.simulation.VisionSystemSim;

public abstract @Data class RobotContainer implements LoggableInputs {
    private TankDriveSubsystem tankDrive;
    private ShooterSubsystem shooterSubsystem;
    private CollectorSubsystem collector;
    private SwerveDriveSubsystem robotSwerveDrive;
    ClimberSubsystem climber;

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
        table.put("Collector", collector);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
        robotSwerveDrive = table.get("SwerveDrive", robotSwerveDrive);
        shooterSubsystem = table.get("Shooter", shooterSubsystem);
        climber = table.get("Climber", climber);
        collector = table.get("Collector", collector);

    }
}