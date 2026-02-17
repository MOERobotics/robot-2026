package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystem.*;
import simulators.ClimberSim;

import static edu.wpi.first.units.Units.Inches;

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

        SparkMax driveMotorFR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFR= new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFR = new CANcoder(32);

        SparkMax driveMotorBL = new SparkMax(19, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBL = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBL = new CANcoder(34);

        SparkMax driveMotorBR = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBR = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder4BR = new CANcoder(33);

        Climber climber = new Climber(new SparkMax(0, SparkLowLevel.MotorType.kBrushless),new PIDConstants(.1,0,0));

        //ClimberSim climberSim = new ClimberSim(climber);

        SwerveModule frontLeftCorner = new SDSSwerveModule(
                driveMotorFL,
                pivotMotorFL,
                swerveModuleEncoderFL,
                Inches.of(4),
                Inches.of(4),
                pivotFeedback,
                driveFeedback
        );
        SwerveModule frontRightCorner = new SDSSwerveModule(
                driveMotorFR,
                pivotMotorFR,
                swerveModuleEncoderFR,
                Inches.of(4),
                Inches.of(-4),
                pivotFeedback,
                driveFeedback
        );
        SwerveModule backLeftCorner = new SDSSwerveModule(
                driveMotorBL,
                pivotMotorBL,
                swerveModuleEncoderBL,
                Inches.of(-4),
                Inches.of(4),
                pivotFeedback,
                driveFeedback
        );
        SwerveModule backRightCorner = new SDSSwerveModule(
                driveMotorBR,
                pivotMotorBR,
                swerveModuleEncoder4BR,
                Inches.of(-4),
                Inches.of(-4),
                pivotFeedback,
                driveFeedback
        );
        SwerveDriveSubsystem SubMOErine = new SDSSwerveDrive(
                robotGyro,
                frontLeftCorner,
                frontRightCorner,
                backLeftCorner,
                backRightCorner
        );


        this.setRobotSwerveDrive(SubMOErine);
        this.setClimber(climber);


    }
}
