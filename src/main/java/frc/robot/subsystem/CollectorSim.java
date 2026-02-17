package frc.robot.subsystem;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CollectorSim {
    private final SparkMax armMotor, wheelMotor;
    private final SparkMaxSim armMotorSim, wheelMotorSim;
    private final DCMotorSim armMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.001, 1.0), DCMotor.getNeo550(1));
    private final DCMotorSim wheelMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.001, 1.0), DCMotor.getNeo550(1));


    public CollectorSim(SparkMax armMotor, SparkMax wheelMotor, SparkMaxSim armMotorSim, SparkMaxSim wheelMotorSim) {
        this.armMotor = armMotor;
        this.wheelMotor = wheelMotor;
        this.armMotorSim = armMotorSim;
        this.wheelMotorSim = wheelMotorSim;
    }
}
