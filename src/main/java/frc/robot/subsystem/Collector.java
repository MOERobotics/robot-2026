package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public class Collector extends MOESubsystem<CollectorInputsAutoLogged> implements CollectorSubsystem, LoggableInputs {

    public SparkMax wheelMotor;
    public SparkMax armMotor;

    private final RelativeEncoder wheelEncoder;
    private final RelativeEncoder armEncoder;

    private final Angle bottomAngle;
    private final Angle topAngle;

    public Collector(SparkMax wheelMotor, SparkMax armMotor, Angle bottomAngle, Angle topAngle) {
        super(new CollectorInputsAutoLogged());
        this.wheelMotor = wheelMotor;
        this.armMotor = armMotor;

        this.wheelEncoder = wheelMotor.getEncoder();
        this.armEncoder = armMotor.getEncoder();

        this.bottomAngle = bottomAngle;
        this.topAngle = topAngle;
        this.setSimulator(new CollectorSim(
                armMotor,
                wheelMotor,
                new SparkMaxSim(armMotor, DCMotor.getNEO(1)),
                new SparkMaxSim(wheelMotor, DCMotor.getNEO(1))
        ));

    }



    @Override
    public CollectorInputs readSensors() {
        CollectorInputs sensors = new CollectorInputs();
        sensors.wheelVelocity = RPM.of(wheelEncoder.getVelocity());
        sensors.collectorArmVelocity = RPM.of(armEncoder.getVelocity());
        sensors.collectorArmAngle = Degrees.of(armEncoder.getPosition());
        sensors.inStartPosition = sensors.collectorArmAngle.lte(bottomAngle);
        sensors.inCollectPosition = sensors.collectorArmAngle.gte(topAngle);
        return sensors;
    }

    @Override
    public void setArmVelocity(AngularVelocity armVelocity) {
        armMotor.set(armVelocity.in(RPM));
    }

    @Override
    public void setRollerVelocity(AngularVelocity wheelVelocity) {
        wheelMotor.set(wheelVelocity.in(RPM));
    }

    @Override
    public Angle getArmAngle() {
        return Degrees.of(armEncoder.getPosition());
    }

    @Override
    public boolean inStartPosition() {
        return getArmAngle().lte(bottomAngle);
    }

    @Override
    public boolean inCollectPosition() {
        return getArmAngle().gte(topAngle);
    }
}
