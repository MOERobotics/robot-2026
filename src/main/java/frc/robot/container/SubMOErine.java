package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystem.SDSSwerveDrive;
import frc.robot.subsystem.SDSSwerveModule;
import frc.robot.subsystem.SwerveDriveSubsystem;
import frc.robot.subsystem.SwerveModule;

public class SubMOErine extends RobotContainer {
    public SubMOErine() {
        double pivotkP = 0.50;
        double pivotkI = 0.001;
        double pivotkD = 0.003;
        double pivotkIMax = 1;

        double drivekP = 1e-3;
        double drivekI = 0;
        double drivekD = 0;

        double drivekS = 0.19959;
        double drivekV = 0.1233;
        double drivekA = 0.019658;

        PIDConstants pivotFeedback = new PIDConstants(pivotkP, pivotkI, pivotkD, pivotkIMax);
        PIDConstants driveFeedback = new PIDConstants(drivekP, drivekI, drivekD);
      //  FeedforwardConstants driveFeedForward = new FeedforwardConstants(drivekS, drivekV, drivekA);

        Pigeon2 robotGyro = new Pigeon2(0);
        SparkMax driveMotorFL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFL = new SparkMax(20, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFL = new CANcoder(31);

        SparkMax driveMotor2FR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor2FR= new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder2FR = new CANcoder(32);

        SparkMax driveMotor3BL = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor3BL = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder3BL = new CANcoder(33);

        SparkMax driveMotor4BR = new SparkMax(19, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor4BR = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder4BR = new CANcoder(34);

        SwerveModule frontLeftCorner = new SDSSwerveModule(
                driveMotorFL,
                pivotMotorFL,
                swerveModuleEncoderFL,
                4,
                4
        );
        SwerveModule frontRightCorner = new SDSSwerveModule(
                driveMotor2FR,
                pivotMotor2FR,
                swerveModuleEncoder2FR,
                4,
                -4
        );
        SwerveModule backLeftCorner = new SDSSwerveModule(
                driveMotor3BL,
                pivotMotor3BL,
                swerveModuleEncoder3BL,
                -4,
                4
        );
        SwerveModule backRightCorner = new SDSSwerveModule(
                driveMotor4BR,
                pivotMotor4BR,
                swerveModuleEncoder4BR,
                -4,
                -4
        );
        SwerveDriveSubsystem SubMOErine = new SDSSwerveDrive(
                robotGyro,
                frontLeftCorner,
                frontRightCorner,
                backLeftCorner,
                backRightCorner
        );
        this.setRobotSwerveDrive(SubMOErine);

}
}
