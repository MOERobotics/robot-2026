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
        sensors.canGoUp = true;//climberSparkMax.getForwardLimitSwitch().isPressed();
        sensors.canGoDown = true;//climberSparkMax.getReverseLimitSwitch().isPressed();
        sensors.velocity = getVelocity();

        // will update later when needed
        //sensors.pidError = 0;
    }

    @Override
    public void stop() {
        setPower(0);
    }

    @Override
    public void setPower(double newPower) {
        // lastVelocity is used to keep a velocity for our periodic fallback
        // also it will tell us the current applied velocity
        getSensors().appliedPower = newPower;

        limits(newPower);
    }

    @Override
    public LinearVelocity getVelocity() {
        return InchesPerSecond.of(climberSparkMax.getEncoder().getVelocity() * 1 * Math.PI / 125);
    }

    @Override
    public Distance getHeight() {
        return Inches.of((climberEncoder.getPosition() * 1 * Math.PI / 125) + minHeight.magnitude());
        //(circumference of spool / gear ratio) + offset
    }
    public void limits(double power){
        if (getSensors().canGoUp && power > 0) {
            climberSparkMax.set(power);
        } else if (getSensors().canGoDown && power < 0) {
            climberSparkMax.set(power);
        } else {
            climberSparkMax.set(0);
        }
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
        limits(getSensors().appliedPower);
        Logger.recordOutput("climberPower", climberSparkMax.get());
    }
}
