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

public class MiniSwerve extends RobotContainer{
    public MiniSwerve(){

        double pivotkP = 1/45.0;
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
        SparkMax driveMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder = new CANcoder(31);

        SparkMax driveMotor2 = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor2= new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder2 = new CANcoder(32);

        SparkMax driveMotor3 = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor3 = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder3 = new CANcoder(33);

        SparkMax driveMotor4 = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotor4 = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoder4 = new CANcoder(34);

        SwerveModule frontLeftCorner = new SDSSwerveModule(driveMotor, pivotMotor, swerveModuleEncoder,4,4, pivotFeedback, driveFeedback);
        SwerveModule frontRightCorner = new SDSSwerveModule(driveMotor2, pivotMotor2, swerveModuleEncoder2,4,-4, pivotFeedback, driveFeedback);
        SwerveModule backLeftCorner = new SDSSwerveModule(driveMotor3, pivotMotor3, swerveModuleEncoder3,-4,4, pivotFeedback, driveFeedback);
        SwerveModule backRightCorner = new SDSSwerveModule(driveMotor4, pivotMotor4, swerveModuleEncoder4,-4,-4,  pivotFeedback, driveFeedback);
        SwerveDriveSubsystem miniSwerve = new SDSSwerveDrive(robotGyro,frontLeftCorner, frontRightCorner, backLeftCorner, backRightCorner);
        this.setRobotSwerveDrive(miniSwerve);

    }

}
