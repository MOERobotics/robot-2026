package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.DriveInputsAutoLogged;
import frc.robot.subsystem.interfaces.TankDriveSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class TankDrive extends MOESubsystem<DriveInputsAutoLogged> implements TankDriveSubsystem {

    public SparkMax motorControlL;
    public SparkMax motorControlR;

    public RelativeEncoder rightEncoder;

    public RelativeEncoder leftEncoder;

    public SparkMaxSim leftMotorSimulator, rightMotorSimulator;

    public SparkRelativeEncoderSim rightEncoderSimulator, leftEncoderSimulator;


    public DCMotorSim leftMotorSystem, rightMotorSystem;

    public DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(14.0));
    Pigeon2 pigeon2 = new Pigeon2(0);


    public DifferentialDriveOdometry driveOdometry;

    @Getter
    public Pose2d simPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));

    Pigeon2SimState pigeonSim;

    DifferentialDriveWheelSpeeds wheelSpeeds= new DifferentialDriveWheelSpeeds(
            0, 0);


    public TankDrive(SparkMax motorControlL, SparkMax motorControlR){
        super(new DriveInputsAutoLogged());
        this.motorControlL = motorControlL;
        this.motorControlR = motorControlR;
        this.leftEncoder = motorControlL.getEncoder();
        this.rightEncoder = motorControlR.getEncoder();
        motorControlR.setInverted(true);
        motorControlL.setInverted(false);


        leftMotorSimulator = new SparkMaxSim(motorControlL, DCMotor.getNEO(1));
        rightMotorSimulator = new SparkMaxSim(motorControlR, DCMotor.getNEO(1));

        leftEncoderSimulator = leftMotorSimulator.getRelativeEncoderSim();
        rightEncoderSimulator = rightMotorSimulator.getRelativeEncoderSim();

        leftMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25
                ),
                DCMotor.getNEO(1)
        );

        rightMotorSystem = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNEO(1),
                        0.005,
                        25
                ),
                DCMotor.getNEO(1)
        );

        pigeonSim = pigeon2.getSimState();

        driveOdometry =  new DifferentialDriveOdometry(pigeon2.getRotation2d(),getLeftPosition(),getRightPosition());
        getSensors().angle = getAngle();
        getSensors().leftPosition = getLeftPosition();
        getSensors().rightPosition = getRightPosition();
        getSensors().simPose = getPose();

    }

    @Override
    public void readSensors(DriveInputsAutoLogged sensors) {
        sensors.leftPosition = getLeftPosition();
        sensors.rightPosition = getRightPosition();
        sensors.angle = getAngle();
        sensors.simPose = this.simPose;
        sensors.pose = getPose();
        sensors.chassisSpeeds = getChassisSpeeds(wheelSpeeds);
        Logger.recordOutput("InchesTraveledPeriodicR", rightEncoder.getPosition() * 4 * Math.PI / 20);
        Logger.recordOutput("InchesTraveledPeriodicL", leftEncoder.getPosition() * 4 * Math.PI / 20);
        Logger.recordOutput("PigeonRotationDegrees",pigeon2.getRotation2d().getMeasure().in(Degrees));
    }


    @Override
    public void drive(double leftPercent, double rightPercent) {
        motorControlL.set(leftPercent);
        motorControlR.set(rightPercent);
        wheelSpeeds.leftMetersPerSecond = leftEncoder.getVelocity()/20*Units.inchesToMeters(4*Math.PI)/60;
        wheelSpeeds.rightMetersPerSecond = rightEncoder.getVelocity()/20*Units.inchesToMeters(4*Math.PI)/60;
    }
    @Override
    public Distance getRightPosition(){
        return Distance.ofRelativeUnits(rightEncoder.getPosition() * 4 * Math.PI / 20, Inches);

    }
    @Override
    public Distance getLeftPosition(){
        return Distance.ofRelativeUnits(leftEncoder.getPosition() * 4 * Math.PI / 20, Inches);

    }

    @Override
    public Angle getAngle(){
        return  pigeon2.getRotation2d().getMeasure();
    }

    @Override
    public void setPose(Pose2d newPose){
         driveOdometry.resetPose(newPose);
    }

    public Pose2d getPose(){
        return driveOdometry.getPoseMeters();
    }

    @Override
    public void simulationPeriodic() {
        // Step 1
        double leftMotorPower = leftMotorSimulator.getSetpoint();
        double rightMotorPower = rightMotorSimulator.getSetpoint();
        // Step 2
        leftMotorSystem.setInputVoltage(leftMotorPower*12.0);
        rightMotorSystem.setInputVoltage(rightMotorPower*12.0);

        leftMotorSystem.update(0.020);
        rightMotorSystem.update(0.020);

        double velocityLeft = leftMotorSystem.getAngularVelocityRPM();
        double velocityRight = rightMotorSystem.getAngularVelocityRPM();

        // Step 3

        Twist2d twist1 = driveKinematics.toTwist2d(
                velocityLeft / 3000.0,
                velocityRight / 3000.0
        );
        pigeonSim.addYaw(Radian.of(twist1.dtheta));

        rightMotorSimulator.iterate(velocityRight*60.0, 12, 0.02);

        leftMotorSimulator.iterate(velocityLeft*60.0, 12, 0.02);
        simPose = driveOdometry.update(
                pigeon2.getRotation2d(),
                new DifferentialDriveWheelPositions(
                        Units.inchesToMeters(getLeftPosition().in(Inches)),
                        Units.inchesToMeters(getRightPosition().in(Inches))
                )
        );
        Logger.recordOutput("updated drive Pos", simPose);

    }
    public ChassisSpeeds getChassisSpeeds (DifferentialDriveWheelSpeeds wheelSpeeds){

        return driveKinematics.toChassisSpeeds(wheelSpeeds);
    }
    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {

        DifferentialDriveWheelSpeeds tempWheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
        double leftPercent = tempWheelSpeeds.leftMetersPerSecond / 3.5; //TODO: find actual max speed in m/s
        double rightPercent = tempWheelSpeeds.rightMetersPerSecond / 3.5;

        drive(leftPercent, rightPercent);
    }


}
