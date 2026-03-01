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

    public SparkMax motorControlL;

    public SparkMax motorControlR;


    public MiniBotContainer() {
        motorControlL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        motorControlR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        this.setTankDrive(new TankDrive(motorControlL, motorControlR));
    }


}
