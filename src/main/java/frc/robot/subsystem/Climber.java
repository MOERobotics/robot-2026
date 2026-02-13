package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;

public class Climber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberSubsystem{
    public final SparkMax climberSparkMax;
    public Distance height;
    public LinearVelocity velocity;
    public boolean canGoUp, canGoDown;

    public Climber( SparkMax climberSparkMax) {
        super(new ClimberInputsAutoLogged());

        this.climberSparkMax = climberSparkMax;

    }
    @Override
    public void readSensors(ClimberInputsAutoLogged sensors){
        sensors.height = height;
        sensors.canGoUp = canGoUp;
        sensors.canGoDown = canGoDown;
        sensors.velocity = velocity;
        sensors.newVelocity = 0; // will update later when needed

    }
    @Override
    public void setVelocity(){}

    @Override
    public void setVelocity(double newVelocity) {

    }

    @Override
    public void getVelocity() {

    }

    @Override
    public Distance getHeight() {
        return null;
    }

    @Override
    public boolean checkCanGoUp() {
        return false;
    }

    @Override
    public boolean checkCanGoDown() {
        return false;
    }


}
