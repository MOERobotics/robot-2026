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
    private double targetTurretAngle = 0;
    private double targetHoodAngle= 0;
    private double targetFlywheelRPM = 0;
    private  Angle TURRET_MIN_ANGLE;
    private  Angle TURRET_MAX_ANGLE;
    private  Angle HOOD_MIN_ANGLE;
    private  Angle HOOD_MAX_ANGLE;
    private static final double TURRET_TOLERANCE = 0;
    private static final double HOOD_TOLERANCE = 0;
    private static final double FLYWHEEL_TOLERANCE = 0;

    public static double TURRET_CONVERSION_FACTOR = 0;
    public static double HOOD_CONVERSION_FACTOR = 0;

    private final SparkMaxConfig flywheelConfig = new SparkMaxConfig();

    private final SparkMaxConfig turretConfig = new SparkMaxConfig();

    private final SparkMaxConfig hoodConfig = new SparkMaxConfig();




    public ShooterControl(SparkMax turretMotor,
                          SparkMax hoodMotor,
                          SparkMax spindexerMotor,
                          SparkMax transitionMotor,
                          SparkMax flywheelMotor,
                          SparkAbsoluteEncoder turretEncoder,
                          SparkAbsoluteEncoder hoodEncoder,
                          Angle TURRET_MIN_ANGLE,
                          Angle TURRET_MAX_ANGLE,
                          Angle HOOD_MIN_ANGLE,
                          Angle HOOD_MAX_ANGLE) {

        super(new ShooterInputsAutoLogged());

        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
        this.spindexerMotor = spindexerMotor;
        this.transitionMotor = transitionMotor;

        this.flywheelMotor = flywheelMotor;


        this.turretEncoder = turretEncoder;
        this.hoodEncoder = hoodEncoder;



        flywheelConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        flywheelMotor.configure(flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);



        turretConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        hoodConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        this.TURRET_MAX_ANGLE = TURRET_MAX_ANGLE;
        this.TURRET_MIN_ANGLE = TURRET_MIN_ANGLE;
        this.HOOD_MAX_ANGLE = HOOD_MAX_ANGLE;
        this.HOOD_MIN_ANGLE = HOOD_MIN_ANGLE;


        getSensors().hoodAngle = getHoodAngleinDegrees();
        getSensors().turretAngle = getTurretAngleinDegrees();
        getSensors().flywheelSpeed = getFlywheelSpeed();
        getSensors().hoodSpeed = getHoodSpeed();
        getSensors().spinDexerSpeed = getSpindexerSpeed();
        getSensors().transitionSpeed= getTransitionSpeed();
        getSensors().transitionOn= getTransitionOn();
        getSensors().spindexerOn= getSpindexerOn();
        getSensors().hoodTargetAngle= getHoodTargetAngle();
        getSensors().turretTargetAngle= getTurretTargetAngle();

    }


    @Override
    public void setTurretTarget(Angle angle) {

        targetTurretAngle = angle.in(Degrees);
    }



    @Override
    public void setHoodTarget(Angle angle) {

        targetHoodAngle = angle.in(Degrees);
    }

    @Override
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

    @Override
    public boolean isTurretAtTarget() {

        return turretPID.atSetpoint();
    }

    @Override
    public boolean isHoodAtTarget() {
        return hoodPID.atSetpoint();


    }

    @Override
    public boolean isFlywheelAtSpeed() {

        return Math.abs(flywheelMotor.getAlternateEncoder().getVelocity()) < targetFlywheelRPM +FLYWHEEL_TOLERANCE;

    }

    @Override
    public boolean isReadyToShoot() {

        return isTurretAtTarget() && isHoodAtTarget() && isFlywheelAtSpeed();
    }


    @Override
    public Angle getTurretAngleinDegrees() {
        return Degrees.of(turretEncoder.getPosition() * TURRET_CONVERSION_FACTOR);
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