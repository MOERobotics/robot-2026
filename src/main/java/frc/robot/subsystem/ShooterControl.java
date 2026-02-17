package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESimulator;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class ShooterControl extends MOESubsystem<ShooterInputsAutoLogged> implements ShooterSubsystem {


    protected final SparkMax turretMotor;
    protected final SparkMax hoodMotor;
    protected final SparkMax spindexerMotor;
    protected final SparkMax transitionMotor;
    protected final SparkMax flywheelMotor;

    protected final SparkAbsoluteEncoder turretEncoder;
    protected final SparkAbsoluteEncoder hoodEncoder;


    private AngularVelocity targetFlywheelRPM = RPM.of(0);
    private Angle TURRET_MIN_ANGLE;
    private Angle TURRET_MAX_ANGLE;
    private Angle HOOD_MIN_ANGLE;
    private Angle HOOD_MAX_ANGLE;
    private static final AngularVelocity FLYWHEEL_TOLERANCE = RPM.of(1);
    private static final Angle HOOD_TOLERANCE = Degree.of(1);
    private static final Angle TURRET_TOLERANCE = Degree.of(1);


    public static double TURRET_CONVERSION_FACTOR = 1;
    public static double HOOD_CONVERSION_FACTOR = 1;

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

        ShooterSimulator shooterSimulator = new ShooterSimulator(this);
        setSimulator(shooterSimulator);


    }


    @Override
    public void periodic() {

        getSensors().hoodAngle = getHoodAngleinDegrees();
        getSensors().turretAngle = getTurretAngleinDegrees();
        getSensors().flywheelSpeed = getFlywheelSpeed();
        getSensors().hoodSpeed = getHoodSpeed();
        getSensors().spinDexerSpeed = getSpindexerSpeed();
        getSensors().transitionSpeed = getTransitionSpeed();
        getSensors().transitionOn = getTransitionOn();
        getSensors().spindexerOn = getSpindexerOn();
        getSensors().reachedMaxHood = reachedHoodMax();
        getSensors().reachedMinHood = reachedHoodMin();
        getSensors().reachedMaxTurret = reachedTurretMax();
        getSensors().reachedMinTurret =reachedTurretMin();

    }

    @Override
    public void setTurretPower(double power) {
        turretMotor.set(power);
    }

    @Override
    public void setHoodPower(double power) {
        hoodMotor.set(power);
    }

    @Override
    public void setFlywheelPower(double power) {
        flywheelMotor.set(power);
    }


    @Override
    public void loadFuel(double spindexerPower, double transitionPower) {
        if (isReadyToShoot()) {
            spindexerMotor.set(spindexerPower);
            transitionMotor.set(transitionPower);
        }
    }

    @Override
    public void setTransitionPower(double transitionPower) {
        transitionMotor.set(transitionPower);
    }

    @Override
    public void setSpindexerPower(double spindexerPower) {
        spindexerMotor.set(spindexerPower);
    }

    public void stopFeeding() {
        spindexerMotor.set(0);
        transitionMotor.set(0);
    }

    @Override
    public boolean isFlywheelAtSpeed() {
        double error = Math.abs( (RPM.of(flywheelMotor.getAlternateEncoder().getVelocity()).minus(targetFlywheelRPM)).in(RPM));
        return RPM.of(error).lt(FLYWHEEL_TOLERANCE);
    }

    @Override
    public boolean isReadyToShoot() {

        return getSpindexerOn() && getTransitionOn() && isFlywheelAtSpeed();
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
    public AngularVelocity getFlywheelSpeed() {
        return RPM.of(flywheelMotor.getAlternateEncoder().getVelocity());
    }

    @Override
    public boolean reachedHoodMax() {
        return this.getHoodAngleinDegrees().gt(Degrees.of(HOOD_MAX_ANGLE.in(Degrees)).minus(Degrees.of(HOOD_TOLERANCE.in(Degrees))));
    }

    @Override
    public boolean reachedHoodMin() {
        return this.getHoodAngleinDegrees().lt(Degrees.of(HOOD_MIN_ANGLE.in(Degrees)).plus(Degrees.of(HOOD_TOLERANCE.in(Degrees))));
    }
    @Override
    public boolean reachedTurretMax() {
        return this.getTurretAngleinDegrees().gt(Degrees.of(TURRET_MAX_ANGLE.in(Degrees)).minus(Degrees.of(TURRET_TOLERANCE.in(Degrees))));
    }
    @Override
    public boolean reachedTurretMin() {
        return this.getTurretAngleinDegrees().lt(Degrees.of(TURRET_MIN_ANGLE.in(Degrees)).plus(Degrees.of(TURRET_TOLERANCE.in(Degrees))));
    }


}