package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public interface ShooterSubsystem extends Subsystem, LoggableInputs {


    @AutoLog
    class ShooterInputs{
        public Angle turretAngle = Degrees.zero();

        public Angle turretAngleThroughbore = Rotation.zero();

        public Angle turretRelativeAngle = Rotation.zero();

        public double turretRelativeAngleDegrees = 0;

        public Angle hoodAngleThroughbore = Rotation.zero();

        public Angle hoodAngleMotor = Rotation.zero();

        public Angle relativeEncoderAngle = Rotation.zero();
        public double flywheelSpeedInRPM = 0;

        public double turretAngleDegrees = 0;

        public double hoodAngleThroughboreDegrees = 0;

        public boolean shooting = false;

        public boolean spindexerOn = false;

        public boolean transitionOn = false;

        public boolean rampOn = false;

        public AngularVelocity flywheelSpeed = RPM.zero();

        public AngularVelocity spindexerSpeed =  RPM.zero();
        public AngularVelocity transitionSpeed =  RPM.zero();

        public AngularVelocity turretSpeed = RPM.zero();

        public AngularVelocity hoodSpeed = RPM.zero();

        public AngularVelocity rampSpeed = RPM.zero();



        public double spindexerPower=  0;

        public double turretPower=  0;
        public double flywheelPower=  0;
        public double transitionPower=  0;
        public double hoodPower=  0;
        public double rampPower=  0;



        public double turretMaxAngle = 0;
        public double turretMinAngle = 0;
        public double hoodMaxAngle = 0;
        public double hoodMinAngle = 0;
        public double adjustedAngle = 0;
        public boolean reachedMaxTurret = false;
        public boolean reachedMinTurret = false;
        public boolean reachedMaxHood = false;
        public boolean reachedMinHood = false;
        public boolean atShooterSpeed = false;

        public double transitionCurrent =0;

        public boolean transitionCurrentLimit =false;

        public Angle offset;

        public boolean hoodAtSetpoint = false;
        public boolean turretAtSetpoint = false;

        public Translation2d turretOffset = new Translation2d();
        public Translation2d turretPosition = new Translation2d();




    }

     public Translation2d turretOffset =  new Translation2d(Inches.of(-6), Inches.of(8.375));

    public ShooterInputs getSensors();


    void setTurretPower(double power);

    void setHoodPower(double power);

    void setFlywheelPower(double power);

    default void loadFuel(double spindexerPower, double transitionPower){
        setTransitionPower(transitionPower);
        setSpindexerPower(spindexerPower);
    };

    void setTransitionPower(double transitionPower);

    void setRampPower(double rampPower);



    void setSpindexerPower(double spindexerPower);


    void stopFeeding();

    default boolean reachedHoodMax(){
        return getSensors().reachedMaxHood;
    };

    default boolean reachedHoodMin(){
        return getSensors().reachedMinHood;
    };

    default boolean reachedTurretMax(){
        return getSensors().reachedMaxTurret;
    };

    default boolean reachedTurretMin(){
        return getSensors().reachedMinTurret;
    };




    default Angle getTurretAngle() {
        return getSensors().turretAngle;
    }

    default Angle getHoodAngleFromThroughbore() {
        return getSensors().hoodAngleThroughbore;
    }

    default Angle getHoodAngleFromMotor() {
        return getSensors().hoodAngleMotor;
    }

    default AngularVelocity getFlywheelSpeed() {
        return getSensors().flywheelSpeed;
    }

    default boolean getSpindexerOn() {
        return getSensors().spindexerOn;
    }

    default boolean getTransitionOn() {
        return getSensors().transitionOn;
    }

    default double feedForwardCalc(double rpm){
        // magic numbers obtained from linear regression model of optimal flywheel rpms & power
        // returns percentage power needed for obtaining inputted rpm
        return ((rpm/5796.35)+0.0487);
    }

    default double calculateShooterSpeed(double distance) {
        return 6.18842*distance+2429.32725;
    }

    default double calcHoodAngle(double distance) {
        return 0.000586636*Math.pow(distance, 2) - (0.0564512*distance) + 187.01223;


    }

    default Translation2d getHubPosition() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isPresent()) {
            isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

        return isRed
                ? new Translation2d(12.286869, 4.034)
                : new Translation2d(4.624, 4.034);
    }



        default Translation2d getTurretPosition(Pose2d pose, Translation2d offset) {
            return pose.getTranslation().plus(offset);
        }

        default double getDistance(Translation2d turretPos, Translation2d target) {
            return Meters.of(turretPos.getDistance(target)).in(Inches);
        }

        default double getTurretAngle(Pose2d pose, Translation2d turretPos, Translation2d target) {
            Rotation2d targetAngle = target.minus(turretPos).getAngle();
            Rotation2d robotHeading = pose.getRotation();

            double angle = targetAngle.minus(robotHeading)
                    .plus(Rotation2d.k180deg)
                    .getDegrees();

            while (angle > 180) angle -= 360;
            while (angle < -180) angle += 360;

            return angle;
        }




}
