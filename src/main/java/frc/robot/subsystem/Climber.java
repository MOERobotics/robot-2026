package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;
import frc.robot.subsystem.interfaces.ClimberSubsystem;
import frc.robot.subsystem.simulations.ClimberSim;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class Climber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberSubsystem {
    public final SparkMax climberSparkMax;

    public RelativeEncoder climberEncoder;

    public  Distance maxHeight;

    public  Distance minHeight;

    public ClimberSim climberSim;


    public Climber(SparkMax climberSparkMax, Distance maxHeight, Distance minHeight) {
        super(new ClimberInputsAutoLogged());


        this.climberSparkMax = climberSparkMax;

        this.climberEncoder = climberSparkMax.getEncoder();

        this.maxHeight = maxHeight;

        this.minHeight = minHeight;
        //this.potentiometer = potentiometer;

        climberSim = new ClimberSim(this);

        setSimulator(climberSim);

    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        sensors.height = getHeight();
        sensors.canGoUp = climberSparkMax.getForwardLimitSwitch().isPressed();
        sensors.canGoDown = climberSparkMax.getReverseLimitSwitch().isPressed();
        sensors.velocity = getVelocity();

        // will update later when needed
        //sensors.pidError = 0;
    }

    @Override
    public void stop() {
        setVelocity(InchesPerSecond.zero());
    }

    @Override
    public void setVelocity(LinearVelocity newVelocity) {
        // lastVelocity is used to keep a velocity for our periodic fallback
        // also it will tell us the current applied velocity
        getSensors().lastVelocity = newVelocity;

        velocityLimits(newVelocity);
    }

    @Override
    public LinearVelocity getVelocity() {
        return InchesPerSecond.of(climberSparkMax.getEncoder().getVelocity());
    }

    @Override
    public Distance getHeight() {
        return Inches.of((climberEncoder.getPosition() * 1 * Math.PI / 125) + minHeight.magnitude());
        //(circumference of spool / gear ratio) + offset
    }
    public void velocityLimits(LinearVelocity velocity){
       /*
        if (getSensors().canGoUp && velocity.gt(InchesPerSecond.zero())) {
            climberSparkMax.set(velocity.in(InchesPerSecond));
        } else if (getSensors().canGoDown && getSensors().lastVelocity.lt(InchesPerSecond.zero())) {
            climberSparkMax.set(velocity.in(InchesPerSecond));
        } else {
            climberSparkMax.set(0);
        }

        */


        climberSparkMax.set(velocity.in(InchesPerSecond));

    }
    @Override
    public void unlatchHooks(){
        getSensors().hooksLatched = false;
    }


    @Override
    public void periodic() {
        super.periodic();
        // this is our fallback system in case setVelocity() stops getting called before we need it to;
        // it just makes sure that the motors can get to their set points and then stop.
        velocityLimits(getSensors().lastVelocity);
        Logger.recordOutput("climberPower", climberSparkMax.get());
    }
}
