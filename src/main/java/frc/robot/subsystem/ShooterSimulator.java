package frc.robot.subsystem;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.MOESimulator;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

public class ShooterSimulator implements MOESimulator {
    public SparkMaxSim hoodMotorSim, turretMotorSim, spindexerMotorSim, transitionMotorSim, flywheelMotorSim;

    public SparkAbsoluteEncoderSim hoodEncoderSim, turretEncoderSim;

    public FlywheelSim flywheelMotorSystem;


    public DCMotorSim hoodMotorSystem, turretMotorSystem, spindexerMotorSystem, transitionMotorSystem;


    public ShooterSimulator(ShooterControl shooter) {
        hoodMotorSim = new SparkMaxSim(shooter.hoodMotor, DCMotor.getNEO(1));
        turretMotorSim = new SparkMaxSim(shooter.turretMotor , DCMotor.getNEO(1));
        spindexerMotorSim = new SparkMaxSim(shooter.spindexerMotor, DCMotor.getNEO(1));
        transitionMotorSim = new SparkMaxSim(shooter.transitionMotor, DCMotor.getNEO(1));
        flywheelMotorSim = new SparkMaxSim(shooter.flywheelMotor, DCMotor.getNEO(1));

        hoodEncoderSim = new SparkAbsoluteEncoderSim(shooter.hoodMotor);
        turretEncoderSim = new SparkAbsoluteEncoderSim(shooter.turretMotor);


        hoodMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25
                ),
                DCMotor.getNEO(1)
        );

        turretMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25
                ),
                DCMotor.getNEO(1)
        );

        spindexerMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25
                ),
                DCMotor.getNEO(1)
        );


        transitionMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25
                ),
                DCMotor.getNEO(1)
        );


        flywheelMotorSystem = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 25), DCMotor.getNEO(1));





    }


    @Override
    public void updateSimState() {
        // Step 1
        double turretPower = turretMotorSim.getSetpoint();
        double hoodPower = hoodMotorSim.getSetpoint();
        double spindexerPower = spindexerMotorSim.getSetpoint();
        double transitionPower = transitionMotorSim.getSetpoint();
        double flyWheelPower = flywheelMotorSim.getSetpoint();


        turretMotorSystem.setInputVoltage(turretPower*12.0);
        hoodMotorSystem.setInputVoltage(hoodPower*12.0);
        spindexerMotorSystem.setInputVoltage(spindexerPower*12.0);
        transitionMotorSystem.setInputVoltage(transitionPower*12.0);
        flywheelMotorSystem.setInputVoltage(flyWheelPower*12.0);


        double velocityTurret = turretMotorSystem.getAngularVelocityRPM();
        double velocityHood = hoodMotorSystem.getAngularVelocityRPM();
        double velocitySpindexer = spindexerMotorSystem.getAngularVelocityRPM();
        double velocityTransition = transitionMotorSystem.getAngularVelocityRPM();
        double velocityFlywheel = flywheelMotorSystem.getAngularVelocityRPM();



        turretMotorSystem.update(0.020);
        hoodMotorSystem.update(0.020);
        spindexerMotorSystem.update(0.020);
        transitionMotorSystem.update(0.020);
        flywheelMotorSystem.update(0.020);


        turretMotorSim.iterate(velocityTurret*60.0, 12, 0.02);

        hoodMotorSim.iterate(velocityHood*60.0, 12, 0.02);

        flywheelMotorSim.iterate(velocityFlywheel*60.0, 12, 0.02);

        transitionMotorSim.iterate(velocityTransition*60.0, 12, 0.02);

        spindexerMotorSim.iterate(velocitySpindexer*60.0, 12, 0.02);


        hoodEncoderSim.setPosition(hoodMotorSystem.getAngularPosition().in(Rotations));
        turretEncoderSim.setPosition(turretMotorSystem.getAngularPosition().in(Rotations));

        turretEncoderSim.setVelocity(turretMotorSystem.getAngularVelocity().in(RPM));
        hoodEncoderSim.setVelocity(hoodMotorSystem.getAngularVelocity().in(RPM));



    }


}
