package frc.robot.subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
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
    private final AbsoluteEncoder armEncoder;

    private final Angle bottomAngle;
    private final Angle topAngle;

    public Collector(SparkMax rollerMotor, SparkMax armMotor, Angle bottomAngle, Angle topAngle) {
        super(new CollectorInputsAutoLogged());
        this.rollerMotor = rollerMotor;
        this.armMotor = armMotor;

        this.rollerEncoder = rollerMotor.getEncoder();
        this.armEncoder = armMotor.getAbsoluteEncoder();

        this.bottomAngle = bottomAngle;
        this.topAngle = topAngle;
        this.setSimulator(new CollectorSim(armMotor, rollerMotor, new SparkMaxSim(armMotor, DCMotor.getNEO(1)), new SparkMaxSim(rollerMotor, DCMotor.getNEO(1))));

    }


    @Override
    public void readSensors(CollectorInputsAutoLogged sensors) {
        sensors.rollerVelocity = RPM.of(rollerEncoder.getVelocity());
        sensors.collectorArmVelocity = RPM.of(armEncoder.getVelocity());
        sensors.collectorArmAngle = Rotations.of(armEncoder.getPosition());
        sensors.collectorArmAngleDegrees = (Rotations.of(armEncoder.getPosition())).in(Degrees);
        sensors.inStartPosition = Rotations.of(armEncoder.getPosition()).gte(topAngle);
        sensors.inCollectPosition = Rotations.of(armEncoder.getPosition()).lte(bottomAngle);
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


        armMotor.set(armVelocity.in(RPM));
        Logger.recordOutput("Top Angle", topAngle);
        Logger.recordOutput("Bottom Angle", bottomAngle);
    }

    @Override
    public void setRollerPower(double power) {
        rollerMotor.set(power);
    }

    @Override
    public AngularVelocity getArmVelocity() {
        return getSensors().collectorArmVelocity;
    }

    @Override
    public Angle getArmAngle() {
        return getSensors().collectorArmAngle;
    }
    @Override
    public boolean inStartPosition() {
        return getSensors().inStartPosition;
    }

    @Override
    public boolean inCollectPosition() {
        return getSensors().inCollectPosition;
    }


}