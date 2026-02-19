package frc.robot.subsystem;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.AnalogInput;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.Logger;
import simulators.ClimberSim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

public class Climber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberSubsystem{
    public final SparkMax climberSparkMax;
    //public final AnalogPotentiometer potentiometer;
    //public Distance height;
    public boolean canGoUp, canGoDown;
    public PIDController pidController;
    public SparkMaxConfig climberMotorConfig;
    public RelativeEncoder climberEncoder;



    public static Distance MAX_HEIGHT = Inches.of(29.75);

    public static Distance MIN_HEIGHT = Inches.of(20.0);;

    public ClimberSim climberSim;






    public Climber(SparkMax climberSparkMax, PIDConstants pidConstants) {
        super(new ClimberInputsAutoLogged());

        this.climberMotorConfig = new SparkMaxConfig();

        this.climberSparkMax = climberSparkMax;

        this.climberEncoder = climberSparkMax.getEncoder();

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


        climberSim = new ClimberSim(this);

        setSimulator(climberSim);

    }
    @Override
    public void readSensors(ClimberInputsAutoLogged sensors){
        sensors.height = getHeight();
        sensors.canGoUp = getHeight().lte(MAX_HEIGHT);
        sensors.canGoDown = getHeight().gte(MIN_HEIGHT);
        sensors.velocity = getVelocity();

        // will update later when needed
        //sensors.pidError = 0;
    }

    @Override
    public void stopVelocity() {
        setVelocity(InchesPerSecond.zero());
    }

    @Override
    public void setVelocity(LinearVelocity newVelocity) {
        // lastVelocity is used to keep a velocity for our periodic fallback
        // also it will tell us the current applied velocity
        getSensors().lastVelocity = newVelocity;

        if (getSensors().canGoUp && newVelocity.gt(InchesPerSecond.zero())){
            climberSparkMax.set(newVelocity.in(InchesPerSecond));
        }
        else if (getSensors().canGoDown && newVelocity.lt(InchesPerSecond.zero())){
            climberSparkMax.set(newVelocity.in(InchesPerSecond));
        }
        else {
            climberSparkMax.set(0);
        }
        /*
            LinearVelocity currentVelocity = InchesPerSecond.of(climberSparkMax.getEncoder().getVelocity()) ;
            LinearVelocity velocityError = currentVelocity.minus(newVelocity);
            climberSparkMax.set(pidController.calculate(velocityError.in(InchesPerSecond))/1);

            getSensors().pidError = pidController.getAccumulatedError();
            getSensors().newVelocity = newVelocity.in(InchesPerSecond);

         */
        // }
        // Logger.recordOutput("stopped", "stopVelocity triggered!");
        // }


    }

    @Override
    public LinearVelocity getVelocity() {
        return InchesPerSecond.of(climberSparkMax.getEncoder().getVelocity());
    }

    @Override
    public Distance getHeight(){
        return Inches.of((climberEncoder.getPosition() * 1 * Math.PI / 125) + 20);
        //(diameter of spool / gear ratio) + offset
    }

    @Override
    public void simulationPeriodic(){
        climberSim.updateSimState();
    }

    @Override
    public void periodic() {
        super.periodic();
        // this is our fallback system in case setVelocity() stops getting called before we need it to;
        // it just makes sure that the motors can get to their set points and then stop.
        if (getSensors().canGoUp && getSensors().lastVelocity.gt(InchesPerSecond.zero())){
            climberSparkMax.set(getSensors().lastVelocity.in(InchesPerSecond));
        }
        else if (getSensors().canGoDown && getSensors().lastVelocity.lt(InchesPerSecond.zero())){
            climberSparkMax.set(getSensors().lastVelocity.in(InchesPerSecond));
        }
        else {
            climberSparkMax.set(0);
        }
    }
}
