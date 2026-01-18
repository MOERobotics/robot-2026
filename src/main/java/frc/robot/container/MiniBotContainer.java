package frc.robot.container;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.TankDrive;
import frc.robot.subsystem.TankDriveSubsystem;
import org.littletonrobotics.junction.LogTable;

public class MiniBotContainer extends RobotContainer {
    public SparkMax motorControlL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

    public SparkMax motorControlR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

    public Pigeon2 pigeon2 = new Pigeon2(0);

    private TankDriveSubsystem tankDrive;
    private PowerDistribution pdh;

    public MiniBotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
        tankDrive = new TankDrive(motorControlL,motorControlR,pigeon2);
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
