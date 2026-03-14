package frc.robot.subsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ShooterInputsAutoLogged;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.simulations.ShooterSimulator;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Shooter extends MOESubsystem<ShooterInputsAutoLogged> implements ShooterSubsystem {


    public final SparkMax turretMotor;
    public final SparkMax hoodMotor;
    public final SparkMax spindexerMotor;
    public final SparkMax transitionMotor;
    public final SparkMax flywheelMotor;

    protected final SparkAbsoluteEncoder turretEncoder;
    protected final SparkAbsoluteEncoder hoodEncoder;


    private final Angle turretMinAngle;
    private final Angle turretMaxAngle;
    private final Angle hoodMinAngle;
    private final Angle hoodMaxAngle;

    private static final Angle HOOD_TOLERANCE = Degree.of(1);
    private static final Angle TURRET_TOLERANCE = Degree.of(1);


    public static double TURRET_CHAIN = 8.266;

    public static double TURRET_CORNER = 100; // corner gear ratio


    public static double HOOD_CONVERSION_FACTOR = (
        Revolutions.of(1)
            .div(90) // VEX Planetaries
            .div(3) // Right Angle Sprocket thingy
            .in(Degrees)
    );


    public Shooter(SparkMax turretMotor,
                   SparkMax hoodMotor,
                   SparkMax spindexerMotor,
                   SparkMax transitionMotor,
                   SparkMax flywheelMotor,
                   SparkAbsoluteEncoder turretEncoder,
                   SparkAbsoluteEncoder hoodEncoder,
                   Angle turretMinAngle,
                   Angle turretMaxAngle,
                   Angle hoodMinAngle,
                   Angle hoodMaxAngle) {

        super(new ShooterInputsAutoLogged());

        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
        this.spindexerMotor = spindexerMotor;
        this.transitionMotor = transitionMotor;

        this.flywheelMotor = flywheelMotor;


        this.turretEncoder = turretEncoder;
        this.hoodEncoder = hoodEncoder;


        this.turretMaxAngle = turretMaxAngle;
        this.turretMinAngle = turretMinAngle;
        this.hoodMaxAngle = hoodMaxAngle;
        this.hoodMinAngle = hoodMinAngle;

        Angle currAngle = getTurretAngle();
        Angle turretAngle = currAngle.minus(Degrees.of(180));

        double adjustedAngle = turretAngle.in(Rotation)*TURRET_CORNER;

        this.turretMotor.getEncoder().setPosition(adjustedAngle);

        ShooterSimulator shooterSimulator = new ShooterSimulator(this);
        setSimulator(shooterSimulator);
    }

    @Override
    public void readSensors(ShooterInputsAutoLogged sensors) {

        getSensors().turretRelativeAngle = Rotations.of(turretMotor.getEncoder().getPosition());
        getSensors().turretAngleThroughbore =  Rotation.of(turretEncoder.getPosition());
        getSensors().hoodAngleThroughbore = getHoodAngleFromThroughbore();
        getSensors().hoodAngleMotor = getHoodAngleFromMotor();
        getSensors().hoodAngleThroughboreDegrees = getHoodAngleFromThroughbore().in(Degrees);
        getSensors().turretAngleDegrees = getTurretAngle().in(Degrees);
        getSensors().turretAngle = getTurretAngle();
        getSensors().flywheelSpeed = getFlywheelSpeed();
        getSensors().hoodSpeed = RPM.of(hoodMotor.getAbsoluteEncoder().getVelocity());
        getSensors().spindexerSpeed = RPM.of(spindexerMotor.getEncoder().getVelocity());
        getSensors().transitionSpeed = RPM.of(transitionMotor.getEncoder().getVelocity());
        getSensors().transitionOn = getTransitionOn();
        getSensors().spindexerOn = getSpindexerOn();
        getSensors().reachedMaxHood = reachedHoodMax();
        getSensors().reachedMinHood = reachedHoodMin();
        getSensors().reachedMaxTurret = reachedTurretMax();
        getSensors().reachedMinTurret =reachedTurretMin();
        getSensors().turretSpeed = RPM.of(turretMotor.getEncoder().getVelocity());
        getSensors().turretMaxAngle = this.turretMaxAngle.in(Degrees);
        getSensors().turretMinAngle = this.turretMinAngle.in(Degrees);
        getSensors().hoodMaxAngle = this.hoodMaxAngle.in(Degrees);
        getSensors().hoodMinAngle = this.hoodMinAngle.in(Degrees);

        getSensors().flywheelPower = flywheelMotor.get();
        getSensors().transitionPower = transitionMotor.get();
        getSensors().spindexerPower = spindexerMotor.get();
        getSensors().hoodPower = hoodMotor.get();
        getSensors().turretPower = turretMotor.get();

        getSensors().spindexerOn = spindexerMotor.get()>0;
        getSensors().transitionOn = transitionMotor.get()>0;

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
        Logger.recordOutput("FlywheelActualPower", power);
        flywheelMotor.set(power);
    }


    @Override
    public void loadFuel(double spindexerPower, double transitionPower) {
            spindexerMotor.set(spindexerPower);
            transitionMotor.set(transitionPower);
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
    public Angle getTurretAngle() {
        return Rotations.of(turretEncoder.getPosition());
    }

    @Override
    public Angle getHoodAngleFromThroughbore() {
        //return Degrees.of(hoodEncoder.getPosition() * HOOD_CONVERSION_FACTOR);
        return Rotation.of(hoodEncoder.getPosition());
    }
    @Override
    public Angle getHoodAngleFromMotor() {
        return Rotations.of(hoodMotor.getEncoder().getPosition()).times(HOOD_CONVERSION_FACTOR);

    }


    public Angle getTurretOffset() {
        Angle currAngle = getTurretAngle();
        Angle relativeEncoder = Rotations.of(turretMotor.getEncoder().getPosition());


        return null;
    }

    @Override
    public AngularVelocity getFlywheelSpeed() {
        return RPM.of(flywheelMotor.getEncoder().getVelocity());
    }


    @Override
    public boolean reachedHoodMax() {
        return this.getHoodAngleFromThroughbore().gt(Degrees.of(hoodMaxAngle.in(Degrees)).minus(Degrees.of(HOOD_TOLERANCE.in(Degrees))));
    }

    @Override
    public boolean reachedHoodMin() {
        return this.getHoodAngleFromThroughbore().lt(Degrees.of(hoodMinAngle.in(Degrees)).plus(Degrees.of(HOOD_TOLERANCE.in(Degrees))));
    }
    @Override
    public boolean reachedTurretMax() {
        return this.getTurretAngle().gt(Degrees.of(turretMaxAngle.in(Degrees)).minus(Degrees.of(TURRET_TOLERANCE.in(Degrees))));
    }
    @Override
    public boolean reachedTurretMin() {
        return this.getTurretAngle().lt(Degrees.of(turretMinAngle.in(Degrees)).plus(Degrees.of(TURRET_TOLERANCE.in(Degrees))));
    }


}