package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.SwerveModuleInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.Amps;

public interface SwerveModuleSubsystem extends Subsystem, LoggableInputs {
    public SwerveModuleState getSpeedNDirectionOfMod();// check if state is correct

    public void setSpeed(double moduleSpeed);

    public void setPivot(Rotation2d modulePivot);
    default void stop(){};

    public Translation2d getCoordsOfModule();

    public SwerveModulePosition getTravelDistanceNRobotAngle();

    public SwerveModuleInputsAutoLogged getSensors();



    @AutoLog
    public class SwerveModuleInputs {
        public Angle moduleAngle;
        public Angle moduleTargetAngle;
        public Translation2d coordOfModule;
        public SwerveModulePosition travelDistanceNRobotAngle;
        public SwerveModuleState speedNDirectionOfMod;
        public double robotPivotSpeed;
        public double robotDriveSpeed;
        public double robotError;

        public double pivotPower = 0;
        public double drivePower = 0;

        public Current pivotAmperage = Amps.of(0);



    }
}


