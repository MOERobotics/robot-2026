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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class ProMOEtheus extends RobotContainer {
    public ProMOEtheus() {

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
        SparkMax driveMotorFL = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFL = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFL = new CANcoder(31);

        SparkMax driveMotorFR = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFR = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFR = new CANcoder(32);

        SparkMax driveMotorBL = new SparkMax(20, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBL = new CANcoder(33);

        SparkMax driveMotorBR = new SparkMax(19, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBR = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBR = new CANcoder(34);

        SwerveModule frontLeftCorner = new SDSSwerveModule(
                driveMotorFL,
                pivotMotorFL,
                swerveModuleEncoderFL,
                Inches.of(13.5),
                Inches.of(13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(-45),
                true
        );
        SwerveModule frontRightCorner = new SDSSwerveModule(
                driveMotorFR,
                pivotMotorFR,
                swerveModuleEncoderFR,
                Inches.of(13.5),
                Inches.of(-13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(45),
                true
        );
        SwerveModule backLeftCorner = new SDSSwerveModule(
                driveMotorBL,
                pivotMotorBL,
                swerveModuleEncoderBL,
                Inches.of(-13.5),
                Inches.of(13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(-135),
                true
        );
        SwerveModule backRightCorner = new SDSSwerveModule(
                driveMotorBR,
                pivotMotorBR,
                swerveModuleEncoderBR,
                Inches.of(-13.5),
                Inches.of(-13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(135),
                true

        );
        SwerveDriveSubsystem ProMOEtheus = new SDSSwerveDrive(
                robotGyro,
                frontLeftCorner,
                frontRightCorner,
                backLeftCorner,
                backRightCorner
        );
        this.setRobotSwerveDrive(ProMOEtheus);

    }
}


