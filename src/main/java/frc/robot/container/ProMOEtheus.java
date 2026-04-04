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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.*;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import frc.robot.subsystem.interfaces.SwerveModuleSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class ProMOEtheus extends RobotContainer {
    public ProMOEtheus() {


        double pivotkP = 0.010;
        double pivotkI = 0;
        double pivotkD = 0;
        double pivotkIMax = 1;

        double drivekP = 1e-3;
        double drivekI = 0;
        double drivekD = 0;

        double drivekS = 0.19959;
        double drivekV = 0.1233;
        double drivekA = 0.019658;


        // due to browning out issue with the pivot motors
        // they got a ramp rate of 2 frames
        // smart current limit of 40-> 30 amps
        // secondary limit of 70 amps (per suggestion)


        PIDConstants pivotFeedback = new PIDConstants(pivotkP, pivotkI, pivotkD, pivotkIMax);
        PIDConstants driveFeedback = new PIDConstants(drivekP, drivekI, drivekD);
        //  FeedforwardConstants driveFeedForward = new FeedforwardConstants(drivekS, drivekV, drivekA);

        Pigeon2 robotGyro = new Pigeon2(35);
        SparkMax driveMotorFL = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFL = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig FLConfigPivot = new SparkMaxConfig();
        FLConfigPivot.openLoopRampRate(0.04).inverted(true).smartCurrentLimit(30).secondaryCurrentLimit(70);
        pivotMotorFL.configure(FLConfigPivot,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotMotorFL.setInverted(true);

        SparkMaxConfig FLConfigDrive = new SparkMaxConfig();
        FLConfigDrive.smartCurrentLimit(40).secondaryCurrentLimit(70).idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveMotorFL.configure(FLConfigDrive,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        CANcoder swerveModuleEncoderFL = new CANcoder(34);

        SparkMax driveMotorFR = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorFR = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig FRConfigPivot = new SparkMaxConfig();
        FRConfigPivot.openLoopRampRate(0.04).inverted(true).smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast).secondaryCurrentLimit(70);;
        pivotMotorFR.configure(FRConfigPivot,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        SparkMaxConfig FRConfigDrive = new SparkMaxConfig();
        FRConfigDrive.smartCurrentLimit(40).secondaryCurrentLimit(70).idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveMotorFR.configure(FRConfigDrive,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotMotorFR.setInverted(true);

        CANcoder swerveModuleEncoderFR = new CANcoder(31);

        SparkMax driveMotorBL = new SparkMax(20, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        pivotMotorBL.setInverted(true);

        SparkMaxConfig BLConfigDrive = new SparkMaxConfig();
        BLConfigDrive.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).secondaryCurrentLimit(70);
        driveMotorBL.configure(BLConfigDrive,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig BLConfigPivot = new SparkMaxConfig();
        BLConfigPivot.openLoopRampRate(0.04).inverted(true).smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast).secondaryCurrentLimit(70);;
        pivotMotorBL.configure(BLConfigPivot,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);



        CANcoder swerveModuleEncoderBL = new CANcoder(33);

        SparkMax driveMotorBR = new SparkMax(19, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivotMotorBR = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);


        SparkMaxConfig BRConfigDrive = new SparkMaxConfig();
        BRConfigDrive.smartCurrentLimit(40).secondaryCurrentLimit(70).idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveMotorBR.configure(BRConfigDrive,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);



        SparkMaxConfig BRConfigPivot = new SparkMaxConfig();
        BRConfigPivot
                .openLoopRampRate(0.04)
                .inverted(true)
                .smartCurrentLimit(30)
                .secondaryCurrentLimit(70)
                .idleMode(SparkBaseConfig.IdleMode.kCoast);

        pivotMotorBR.configure(BRConfigPivot,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        pivotMotorBR.setInverted(true);
        CANcoder swerveModuleEncoderBR = new CANcoder(32);

        SwerveModuleSubsystem frontLeftCorner = new SDSSwerveModule(
                driveMotorFL,
                pivotMotorFL,
                swerveModuleEncoderFL,
                Inches.of(13.5),
                Inches.of(13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(-45)
        );
        SwerveModuleSubsystem frontRightCorner = new SDSSwerveModule(
                driveMotorFR,
                pivotMotorFR,
                swerveModuleEncoderFR,
                Inches.of(13.5),
                Inches.of(-13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(45)
        );
        SwerveModuleSubsystem backLeftCorner = new SDSSwerveModule(
                driveMotorBL,
                pivotMotorBL,
                swerveModuleEncoderBL,
                Inches.of(-13.5),
                Inches.of(13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(-135)
        );
        SwerveModuleSubsystem backRightCorner = new SDSSwerveModule(
                driveMotorBR,
                pivotMotorBR,
                swerveModuleEncoderBR,
                Inches.of(-13.5),
                Inches.of(-13.5),
                pivotFeedback,
                driveFeedback,
                Degrees.of(135)

        );
        SwerveDriveSubsystem ProMOEtheus = new SDSSwerveDrive(
                robotGyro,
                frontLeftCorner,
                frontRightCorner,
                backLeftCorner,
                backRightCorner
        );

    SparkMax climberMotor = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //climberMotorConfig.
    climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    Climber climber = new Climber(climberMotor, Inches.of(29.75), Inches.of(20.0)) {
    };
        SparkMax collectorArmMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless); // not confrimed arm id
        SparkMax collectorRollerMotor= new SparkMax(17, SparkLowLevel.MotorType.kBrushless); // not confrimed arm id
        collectorRollerMotor.setInverted(true);
        collectorArmMotor.setInverted(true);

       // collectorArmMotor.configAccessor.


        SparkMaxConfig collectorArmConfig = new SparkMaxConfig();

        // also put current limit on collector (forget why though)
        collectorArmConfig.smartCurrentLimit(40);
        collectorArmConfig.inverted(true);
        collectorArmMotor.configure(collectorArmConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        Angle collectorArmBottom = Degrees.of(135);
        Angle collectorArmTop = Degrees.of(215);

        this.setCollector(new Collector(collectorRollerMotor, collectorArmMotor, collectorArmBottom, collectorArmTop));






        SparkMax turretMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

        SparkMax hoodMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);

        SparkMax spindexerMotor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);

        SparkMax transitionMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);

        SparkMax flywheelMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);

        SparkMax rampMotor = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);



        SparkMaxConfig flywheelConfig = new SparkMaxConfig();

        SparkMaxConfig turretConfig = new SparkMaxConfig();

        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        SparkMaxConfig spindexerConfig = new SparkMaxConfig();

        SparkMaxConfig transitionConfig = new SparkMaxConfig();

        SparkMaxConfig rampConfig = new SparkMaxConfig();


        flywheelConfig.idleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(40);
        flywheelConfig.inverted(false);
        flywheelMotor.configure(flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        turretConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        turretConfig.inverted(false);
        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        hoodConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        hoodConfig.inverted(true);
        hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        spindexerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        spindexerConfig.inverted(true);
        spindexerMotor.configure(spindexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        transitionConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        transitionConfig.inverted(false)/*.smartCurrentLimit(20)*/;

        transitionMotor.configure(transitionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        rampConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rampConfig.inverted(true);
        rampMotor.configure(rampConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        Shooter shooter = new Shooter(
                turretMotor,
                hoodMotor,
                spindexerMotor,
                transitionMotor,
                flywheelMotor,
                rampMotor,
                turretMotor.getAbsoluteEncoder(),
                hoodMotor.getAbsoluteEncoder(),
                Degrees.of(-45),
                Degrees.of(45),
                Degrees.of(175),
                Degrees.of(205));

        this.setRobotSwerveDrive(ProMOEtheus);
        this.setClimber(climber);







        this.setCollector(new Collector(collectorRollerMotor, collectorArmMotor, collectorArmBottom, collectorArmTop));
        this.setShooterSubsystem(shooter);
        this.setRobotSwerveDrive(ProMOEtheus);
        this.setClimber(climber);

        // started logging PDH info tbd
        var pdh = new PowerDistribution(21, PowerDistribution.ModuleType.kRev);


        this.setPdh(pdh);

    }
}


