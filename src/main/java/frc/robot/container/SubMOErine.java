package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystem.*;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.SwerveModule;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class SubMOErine extends RobotContainer {
    public SubMOErine() {
        double pivotkP = 0.005;
        double pivotkI = 0.001;
        double pivotkD = 0.00003;
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

        SparkMax driveMotorFR = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderFR = new CANcoder(32);

        SparkMax driveMotorBL = new SparkMax(19, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBL = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBL = new CANcoder(34);

        SparkMax driveMotorBR = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBR = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);
        CANcoder swerveModuleEncoderBR = new CANcoder(33);

        SwerveModule frontLeftCorner = new SDSSwerveModule(
                driveMotorFL,
                pivotMotorFL,
                swerveModuleEncoderFL,
                Inches.of(14.5),
                Inches.of(14.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(-45)
        );
        SwerveModule frontRightCorner = new SDSSwerveModule(
                driveMotorFR,
                pivotMotorFR,
                swerveModuleEncoderFR,
                Inches.of(14.5),
                Inches.of(-14.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(45)
        );
        SwerveModule backLeftCorner = new SDSSwerveModule(
                driveMotorBL,
                pivotMotorBL,
                swerveModuleEncoderBL,
                Inches.of(-14.5),
                Inches.of(14.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(-135)
        );
        SwerveModule backRightCorner = new SDSSwerveModule(
                driveMotorBR,
                pivotMotorBR,
                swerveModuleEncoderBR,
                Inches.of(-14.5),
                Inches.of(-14.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(135)
        );
        SwerveDriveSubsystem SubMOErine = new SDSSwerveDrive(
                robotGyro,
                frontLeftCorner,
                frontRightCorner,
                backLeftCorner,
                backRightCorner
        );
        this.setRobotSwerveDrive(SubMOErine);


        SparkMax turretMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

        SparkMax hoodMotor = new SparkMax(26, SparkLowLevel.MotorType.kBrushless);

        SparkMax spindexerMotor = new SparkMax(27, SparkLowLevel.MotorType.kBrushless);

        SparkMax transitionMotor = new SparkMax(28, SparkLowLevel.MotorType.kBrushless);

        SparkMax flywheelMotor = new SparkMax(29, SparkLowLevel.MotorType.kBrushless);


        SparkMaxConfig flywheelConfig = new SparkMaxConfig();

        SparkMaxConfig turretConfig = new SparkMaxConfig();

        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        SparkMaxConfig spindexerConfig = new SparkMaxConfig();

        SparkMaxConfig transitionConfig = new SparkMaxConfig();



        flywheelConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        flywheelConfig.inverted(false);
        flywheelMotor.configure(flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        turretConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        turretConfig.inverted(false);
        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        hoodConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        hoodConfig.inverted(false);
        hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        spindexerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        spindexerConfig.inverted(true);
        spindexerMotor.configure(spindexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        transitionConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        transitionConfig.inverted(false);
        transitionMotor.configure(transitionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);




        Shooter shooter = new Shooter(
                turretMotor,
                hoodMotor,
                spindexerMotor,
                transitionMotor,
                flywheelMotor,
                turretMotor.getAbsoluteEncoder(),
                hoodMotor.getAbsoluteEncoder(),
                Degrees.of(-5),
                Degrees.of(10),
                Degrees.of(-5),
                Degrees.of(10));
        this.setShooterSubsystem(shooter);

    }


}
