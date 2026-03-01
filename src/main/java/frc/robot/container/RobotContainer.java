package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.TankDriveSubsystem;
import frc.robot.subsystem.interfaces.ClimberSubsystem;
import frc.robot.subsystem.interfaces.CollectorSubsystem;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystem.TankDrive;
import frc.robot.subsystem.TankDriveSubsystem;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract @Data class RobotContainer implements LoggableInputs {
    private TankDriveSubsystem tankDrive;
    ClimberSubsystem climber;
    private ShooterSubsystem shooterSubsystem;
    private SwerveDriveSubsystem robotSwerveDrive;
    private PowerDistribution pdh;
    CollectorSubsystem fuelCollector;

    public RobotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
    }

    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("Shooter", shooterSubsystem);
        table.put("SwerveDrive", robotSwerveDrive);
        table.put ("FuelCollector", fuelCollector);

        table.put("Climber", climber);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
        robotSwerveDrive = table.get("SwerveDrive", robotSwerveDrive);
        fuelCollector = table.get("FuelCollector", fuelCollector);
        climber = table.get("Climber", climber);
        shooterSubsystem = table.get("Shooter", shooterSubsystem);
    }
}
