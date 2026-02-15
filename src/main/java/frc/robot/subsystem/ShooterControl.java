package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class ShooterControl extends MOESubsystem <ShooterInputsAutoLogged> implements ShooterSubsystem{


    private final SparkMax turretMotor;
    private final SparkMax hoodMotor;
    private final SparkMax spindexerMotor;
    private final SparkMax transitionMotor;
    private final SparkMax flywheelMotor;

    private final SparkAbsoluteEncoder turretEncoder;
    private final SparkAbsoluteEncoder hoodEncoder;

    private final PIDController turretPID = new PIDController(0.02, 0, 0);
    private final PIDController hoodPID = new PIDController(0.02, 0, 0);


    private double targetTurretDeg = 0;
    private double targetHoodDeg = 0;
    private double targetFlywheelRPM = 0;


    private static final double TURRET_MIN = -90;
    private static final double TURRET_MAX = 90;

    private static final double HOOD_MIN = 0;
    private static final double HOOD_MAX = 0;

    private static final double TURRET_TOLERANCE = 1.5;
    private static final double HOOD_TOLERANCE = 1.0;
    private static final double FLYWHEEL_TOLERANCE = 150;


    public static double TURRET_CONVERSION_FACTOR = 360.0;
    public static double HOOD_CONVERSION_FACTOR = 360.0;

    public ShooterControl(SparkMax turretMotor, SparkMax hoodMotor, SparkMax spindexerMotor, SparkMax transitionMotor, SparkMax flywheelMotor, SparkAbsoluteEncoder turretEncoder, SparkAbsoluteEncoder hoodEncoder) {

        super(new ShooterInputsAutoLogged());

        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
        this.spindexerMotor = spindexerMotor;
        this.transitionMotor = transitionMotor;

        this.flywheelMotor = flywheelMotor;


        this.turretEncoder = turretEncoder;
        this.hoodEncoder = hoodEncoder;




        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        flywheelMotor.configure(flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }



    public void setTurretTarget(Angle angle) {
        targetTurretDeg = angle.in(Degrees);
    }

    public void setHoodTarget(Angle angle) {
        targetHoodDeg = angle.in(Degrees);
    }

    public void setFlywheelTargetRPM(double rpm) {
        targetFlywheelRPM = rpm;
    }


    @Override
    public void loadFuel(double spindexerPower, double transitionPower) {
        if (isReadyToShoot()) {
            spindexerMotor.set(spindexerPower);
            transitionMotor.set(transitionPower);
        }
    }

    public void stopFeeding() {
        spindexerMotor.set(0);
        transitionMotor.set(0);
    }


    public boolean isTurretAtTarget() {
        return turretPID.atSetpoint();
    }

    public boolean isHoodAtTarget() {
        return hoodPID.atSetpoint();


    }

    public boolean isFlywheelAtSpeed() {

        return Math.abs(flywheelMotor.getAlternateEncoder().getVelocity()) < targetFlywheelRPM +FLYWHEEL_TOLERANCE;


    }

    public boolean isReadyToShoot() {
        return isTurretAtTarget() && isHoodAtTarget() && isFlywheelAtSpeed();
    }


    @Override
    public Angle getTurretAngleinDegrees() {
        return Degrees.of(turretEncoder.getPosition() * TURRET_CONVERSION_FACTOR
        );
    }

    @Override
    public Angle getHoodAngleinDegrees() {
        return Degrees.of(hoodEncoder.getPosition() * HOOD_CONVERSION_FACTOR);
}
    @Override
    public LinearVelocity getFlywheelSpeed() {
        return MetersPerSecond.of(flywheelMotor.getAlternateEncoder().getVelocity());
    }
}