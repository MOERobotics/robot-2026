package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystem.TankDrive;
import frc.robot.subsystem.TankDriveSubsystem;
import frc.robot.subsystem.Vision;
import lombok.Data;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.pathplanner.lib.config.RobotConfig;
import org.photonvision.simulation.VisionSystemSim;

public abstract @Data class RobotContainer implements LoggableInputs {
   // private TankDriveSubsystem tankDrive;
    private PowerDistribution pdh;




    public RobotContainer() {

    }

    public TankDriveSubsystem getTankDrive() {
        return null;
    }
/*
    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("rightPower", tankDrive);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
    }

 */
}
