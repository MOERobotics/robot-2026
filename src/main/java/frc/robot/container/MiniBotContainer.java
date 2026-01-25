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

    public TankDriveSubsystem tankDrive;
    private PowerDistribution pdh;

    public MiniBotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());

    }


}
