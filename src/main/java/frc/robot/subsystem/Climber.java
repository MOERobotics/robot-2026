package frc.robot.subsystem;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.AnalogInput;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class Climber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberSubsystem{
    public final SparkMax climberSparkMax;
    //public final AnalogPotentiometer potentiometer;
    //public Distance height;
    public boolean canGoUp, canGoDown;
    public PIDController pidController;
    public SparkMaxConfig climberMotorConfig;

    public Distance MAX_HEIGHT;
    public Distance MIN_HEIGHT;






    public Climber(SparkMax climberSparkMax, PIDConstants pidConstants) {
        super(new ClimberInputsAutoLogged());

        this.climberMotorConfig = new SparkMaxConfig();

        this.climberSparkMax = climberSparkMax;
        //this.potentiometer = potentiometer;

        this.climberSparkMax.configure(
                climberMotorConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        // decision to be made if we define pid constants here or outside the subsystem
        pidController =
                new PIDController(
                pidConstants.kP,
                pidConstants.kI,
                pidConstants.kD);

        MAX_HEIGHT = Inches.of(29.75);
        MIN_HEIGHT = Inches.of(20.0);

    }
    @Override
    public void readSensors(ClimberInputsAutoLogged sensors){
        //sensors.height = getHeight();
        sensors.canGoUp = canGoUp;
        sensors.canGoDown = canGoDown;
        sensors.velocity = getVelocity();
        sensors.newVelocity = 0; // will update later when needed
        sensors.pidError = pidController.getAccumulatedError();
    }

    @Override
    public void stopVelocity() {
        climberSparkMax.set(0);
    }

    @Override
    public void setVelocity(LinearVelocity newVelocity) {
        if ((canGoUp && newVelocity.gt(InchesPerSecond.zero())) || (canGoDown && newVelocity.lt(InchesPerSecond.zero()))){
            LinearVelocity currentVelocity = InchesPerSecond.of(climberSparkMax.getEncoder().getVelocity()) ;
            LinearVelocity velocityError = currentVelocity.minus(newVelocity);
            climberSparkMax.set(pidController.calculate(velocityError.in(InchesPerSecond)));
        }
        else{
            stopVelocity();
        }
    }

    @Override
    public LinearVelocity getVelocity() {
       return InchesPerSecond.of(climberSparkMax.getEncoder().getVelocity());
    }







}
