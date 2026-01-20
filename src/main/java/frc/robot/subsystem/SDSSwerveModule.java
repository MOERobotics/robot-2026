package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;

import java.util.Base64;

import static edu.wpi.first.hal.simulation.AnalogGyroDataJNI.getAngle;
import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;

public class SDSSwerveModule implements SwerveModule {
    private double  speedWeWant = 0;
     private double pivotWeWant=0;
    private final SparkMax pivotMotor;
    private final SparkMax driveMotor;
    private final CANcoder swerveModuleEncoder;
    public double xCordinate =0; // distance from robot center?;
    public double yCordinate =0; //distance from robot center?;

    public SDSSwerveModule (SparkMax driveMotor, SparkMax pivotMotor, CANcoder swerveModuleEncoder) {

        this.driveMotor = driveMotor;
        this.pivotMotor = pivotMotor;
        this.swerveModuleEncoder = swerveModuleEncoder;

    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(InchesPerSecond.of(driveMotor.getEncoder().getVelocity()*(4* PI / (60.0*6.75))).in(MetersPerSecond),
                new Rotation2d(getAngle()));
    }

        public Angle getAngle() {
            return this.swerveModuleEncoder.getPosition().getValue();
        }


    @Override
    public void setSpeed(double moduleSpeed) {4
        //driveMotor.set(speedWeWant);
        driveMotor.set(moduleSpeed);
    }

    @Override

    public void setPivot(Rotation2d modulePivot) {
        pivotMotor.set(pivotWeWant);
    }

    public void setSpeedWeWant(double speed) {
        speedWeWant = speed;
    }

    public void setPivotWeWant(double pivot) {
        pivotWeWant = pivot;
    }
    @Override
    public Translation2d getTranslation() {
        return new Translation2d(this.xCordinate, this.yCordinate);

    }

    @Override
    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(
                Inches.of(
                driveMotor.getEncoder().getPosition()*(4* PI / 6.75)
                ).in(Meters),
                new Rotation2d(
                    getAngle()
                )
        );
    }
}

