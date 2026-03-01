package frc.robot.subsystem.simulations;


import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.MOESimulator;
import frc.robot.subsystem.SDSSwerveDrive;


public class SwerveDriveSim implements MOESimulator {
    private final SDSSwerveDrive swerveDrive;
    private final Pigeon2SimState pigeon2Sim;
    private SwerveModulePosition[] oldModulePositions, currentModulePositions;

    public SwerveDriveSim(SDSSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        this.pigeon2Sim = swerveDrive.robotGyro.getSimState();
        this.oldModulePositions = new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

    }


    @Override
    public void updateSimState() {
        currentModulePositions = swerveDrive.getSensors().modulePositions.clone();

        Twist2d twist = swerveDrive.robotKinematics.toTwist2d(oldModulePositions, currentModulePositions);
        pigeon2Sim.addYaw(Units.radiansToDegrees(twist.dtheta));

        oldModulePositions = currentModulePositions.clone();
    }

    public void simulationPeriodic() {
        updateSimState();
    }


}