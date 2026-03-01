package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.CollectorInputsAutoLogged;
import frc.robot.subsystem.interfaces.CollectorSubsystem;
import frc.robot.subsystem.simulations.CollectorSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public class Collector extends MOESubsystem<CollectorInputsAutoLogged> implements CollectorSubsystem, LoggableInputs {

    public SparkMax rollerMotor;
    public SparkMax armMotor;

    private final RelativeEncoder rollerEncoder;
    private final RelativeEncoder armEncoder;

    private final Angle bottomAngle;
    private final Angle topAngle;

    public Collector(SparkMax rollerMotor, SparkMax armMotor, Angle bottomAngle, Angle topAngle) {
        super(new CollectorInputsAutoLogged());
        this.rollerMotor = rollerMotor;
        this.armMotor = armMotor;

        this.rollerEncoder = rollerMotor.getEncoder();
        this.armEncoder = armMotor.getEncoder();

        this.bottomAngle = bottomAngle;
        this.topAngle = topAngle;
        this.setSimulator(new CollectorSim(armMotor, rollerMotor, new SparkMaxSim(armMotor, DCMotor.getNEO(1)), new SparkMaxSim(rollerMotor, DCMotor.getNEO(1))));

    }


    @Override
    public void readSensors(CollectorInputsAutoLogged sensors) {
        sensors.rollerVelocity = RPM.of(rollerEncoder.getVelocity());
        sensors.collectorArmVelocity = RPM.of(armEncoder.getVelocity());
        sensors.collectorArmAngle = Degrees.of(armEncoder.getPosition());
        sensors.inStartPosition = sensors.collectorArmAngle.lte(bottomAngle);
        sensors.inCollectPosition = sensors.collectorArmAngle.gte(topAngle);
        sensors.armMotorPower = armMotor.get();
        sensors.rollerMotorPower = rollerMotor.get();
    }

    @Override
    public void setArmVelocity(AngularVelocity armVelocity) {
        if (inStartPosition() && armVelocity.gt(RPM.zero())) { //can't go up
            armMotor.set(0);
        } else if (inCollectPosition() && armVelocity.lte(RPM.zero())) { //can't go down
            armMotor.set(0);
        } else {
            Logger.recordOutput("ArmVelocityRPM", armVelocity);
            armMotor.set(armVelocity.in(RPM));
        }

    }

    @Override
    public void setRollerVelocity(AngularVelocity rollerVelocity) {
        rollerMotor.set(rollerVelocity.in(RPM));
    }

    @Override
    public AngularVelocity getArmVelocity() {
        return getSensors().collectorArmVelocity;
    }

    @Override
    public Angle getArmAngle() {
        return Degrees.of(armEncoder.getPosition());
    }
    @Override
    public boolean inStartPosition() {
        return getArmAngle().gte(topAngle);
    }

    @Override
    public boolean inCollectPosition() {
        return getArmAngle().lte(bottomAngle);
    }
}