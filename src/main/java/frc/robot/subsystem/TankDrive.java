package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.LogTable;

import static edu.wpi.first.units.Units.Inches;

public class TankDrive extends MOESubsystem<DriveInputsAutoLogged> implements TankDriveSubsystem{

    public SparkMax motorControlL;
    public SparkMax motorControlR;

    public RelativeEncoder rightEncoder;

    public RelativeEncoder leftEncoder;



    public TankDrive(SparkMax motorControlL, SparkMax motorControlR){
        super(new DriveInputsAutoLogged());
        this.motorControlL = motorControlL;
        this.motorControlR = motorControlR;
        this.leftEncoder = motorControlL.getEncoder();
        this.rightEncoder = motorControlR.getEncoder();


        motorControlR.setInverted(true);
        motorControlL.setInverted(false);
        getSensors().angle = getAngle();
        getSensors().leftPosition = getLeftPosition();
        getSensors().rightPosition = getRightPosition();


    }

    @Override
    public void readSensors(DriveInputsAutoLogged sensors) {
        sensors.leftPosition = getLeftPosition();
        sensors.rightPosition = getRightPosition();
        sensors.angle = getAngle();

    }

    @Override
    public void drive(double leftPercent, double rightPercent) {
        motorControlL.set(leftPercent);
        motorControlR.set(rightPercent);
    }
    @Override
    public Distance getRightPosition(){
        return Distance.ofRelativeUnits(rightEncoder.getPosition() * 4 * Math.PI / 20, Inches);

    }
    @Override
    public Distance getLeftPosition(){
        return Distance.ofRelativeUnits(leftEncoder.getPosition() * 4 * Math.PI / 20, Inches);

    }
    /**
    @Override
    public Angle getAngle(){
        return  pigeon2.getRotation2d().getMeasure();

    }
     **/
    @Override
    public Pose2d setPose(Pose2d newPose){
        return null;
    }

}
