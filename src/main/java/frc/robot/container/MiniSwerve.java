package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.subsystem.SDSSwerveDrive;
import frc.robot.subsystem.SDSSwerveModule;
import frc.robot.subsystem.SwerveDriveSubsystem;
import frc.robot.subsystem.SwerveModule;

import static edu.wpi.first.units.Units.*;

public class MiniSwerve extends RobotContainer {
    public MiniSwerve() {

        double pivotkP = 1 / 45.0;
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
        //FeedforwardConstants driveFeedForward = new FeedforwardConstants(drivekS, drivekV, drivekA);


        Pigeon2 robotGyro = new Pigeon2(0);
        SparkMax driveMotorFL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFL = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFL = new CANcoder(31);

        SparkMax driveMotorFR = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFR = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFR = new CANcoder(32);

        SparkMax driveMotorBL = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBL = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBL = new CANcoder(33);

        SparkMax driveMotorBR = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBR = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBR = new CANcoder(34);

        SwerveModule frontLeftCorner = new SDSSwerveModule(driveMotorFL, pivotMotorFL, swerveModuleEncoderFL, Inches.of(4), Inches.of(4), pivotFeedback, driveFeedback, Degrees.of(0)
                );
        SwerveModule frontRightCorner = new SDSSwerveModule(driveMotorFR, pivotMotorFR, swerveModuleEncoderFR, Inches.of(4), Inches.of(-4), pivotFeedback, driveFeedback, Degrees.of(0));
        SwerveModule backLeftCorner = new SDSSwerveModule(driveMotorBL, pivotMotorBL, swerveModuleEncoderBL, Inches.of(-4), Inches.of(4), pivotFeedback, driveFeedback, Degrees.of(0));
        SwerveModule backRightCorner = new SDSSwerveModule(driveMotorBR, pivotMotorBR, swerveModuleEncoderBR, Inches.of(-4), Inches.of(-4), pivotFeedback, driveFeedback, Degrees.of(0));
        SwerveDriveSubsystem miniSwerve = new SDSSwerveDrive(robotGyro, frontLeftCorner, frontRightCorner, backLeftCorner, backRightCorner);
        this.setRobotSwerveDrive(miniSwerve);

    }

}
