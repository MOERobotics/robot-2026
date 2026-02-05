package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystem.TankDrive;
import frc.robot.subsystem.TankDriveSubsystem;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract @Data class RobotContainer implements LoggableInputs {
    public TankDriveSubsystem tankDrive;
    private PowerDistribution pdh;
    public SparkMax motorControlL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

    public SparkMax motorControlR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);



    public RobotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
        tankDrive = new TankDrive(motorControlL, motorControlR);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("rightPower", tankDrive);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
    }
}
