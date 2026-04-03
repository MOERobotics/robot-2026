package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ShooterAutoCommand extends Command {


    private ShooterSubsystem shooter;
    private SwerveDriveSubsystem drive;

    private Target selectedTarget;
    private Translation2d targetPosition;

    // copied from teleop stuff


    // TODO tune PID for flywheel/shooter thingy
    public double kP = 2.0 / 1500.0;
    public double kI = 0.00015;
    public double kD = 0.4 / 14000;
    private PIDController flywheelPID = new PIDController(kP, kI, kD);
    private PIDController turretPID = new PIDController(0.0055, 0, 0);
    private PIDController hoodPID = new PIDController(0.056, 0, 0);
    private Timer shootTimer = new Timer();
    boolean feeding = false;

    private double flywheelRPM = 3510;

    public enum Target {
        HUB,
        DEPOT,
        OUTPOST
    }

    boolean justShoot;

    public ShooterAutoCommand(RobotContainer robot) {

        this.shooter = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();

        flywheelPID.setTolerance(300);
        turretPID.setTolerance(1.0);
        hoodPID.setTolerance(1.0);

        this.justShoot = justShoot;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shootTimer.reset();
        shootTimer.stop();
        feeding = false;
    }

    @Override
    public void execute() {

        // TODO fix all this up  because rn it's all messy and hardcoded with angle values
        // TODO make an agitate function in auto?? for jamming?
        Pose2d pose = drive.getPose();

        Translation2d turretOffset = new Translation2d(Inches.of(-2.172), Inches.of(8.4375)).rotateBy(pose.getRotation());
        Translation2d turretPosition = pose.getTranslation().plus(turretOffset);


        Logger.recordOutput("turretOffset", turretOffset);
        Logger.recordOutput("turretPosition", turretPosition);


        double distance = Meters.of(turretPosition.getDistance(getHubPosition())).in(Inches);



        double flywheelSetpoint = calculateShooterSpeed(distance);


        flywheelPID.setSetpoint(flywheelSetpoint);

        double flywheelOutput = flywheelPID.calculate(shooter.getFlywheelSpeed().in(RPM));

        double feedforward = shooter.feedForwardCalc(flywheelSetpoint);
        double outputMax = 1 - feedforward;

        if (flywheelOutput > outputMax) flywheelOutput = outputMax;
        if (flywheelOutput < 0) flywheelOutput = 0;

        Logger.recordOutput("flywheelOutput", flywheelOutput);

        shooter.setFlywheelPower(feedforward + flywheelOutput);





        Rotation2d targetTurretAngle = getHubPosition().minus(turretPosition).getAngle();





        Rotation2d robotHeading = pose.getRotation();

        Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading).plus(Rotation2d.k180deg);

        double desiredTurretAngle = desiredTurret.getDegrees();
        while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
        while (desiredTurretAngle < -180) desiredTurretAngle += 360;





        double currTurretAngle = shooter.getSensors().turretRelativeAngle.in(Degrees);


        double desiredAngle = desiredTurret.getDegrees();





        double desTurretAngle = MathUtil.clamp(desiredAngle, shooter.getSensors().turretMinAngle,shooter.getSensors().turretMaxAngle);

        turretPID.setSetpoint(desTurretAngle);

        double turretOutput = turretPID.calculate(currTurretAngle);




       if( turretOutput >0.3) {
           turretOutput=0.3;
        }




       // shooter.setTurretPower(turretOutput);


        Logger.recordOutput("turretOutput", turretOutput);
        Logger.recordOutput("turretDesiredAngle", desiredTurret.getDegrees());
        Logger.recordOutput("turretCurrentAngle", targetTurretAngle.getDegrees());




        double currHoodAngle = shooter.getHoodAngleFromThroughbore().in(Degrees);

        double desHoodAngle = MathUtil.clamp(calcHoodAngle(distance), shooter.getSensors().hoodMinAngle,shooter.getSensors().hoodMaxAngle);


        hoodPID.setSetpoint(desHoodAngle);

        double hoodOutput = hoodPID.calculate(currHoodAngle+25);

        hoodOutput = MathUtil.clamp(hoodOutput, -0.32, 0.32);


        shooter.setHoodPower(hoodOutput);


        boolean flywheelReady = flywheelPID.atSetpoint();


        boolean turretReady = turretPID.atSetpoint();


        boolean hoodReady = hoodPID.atSetpoint();


        Logger.recordOutput("flywheelReady", flywheelReady);
        Logger.recordOutput("turretReady", turretReady);
        Logger.recordOutput("hoodReady", hoodReady);


        if (flywheelReady /*&& turretReady */ && hoodReady) {

            if (!feeding) {
                shootTimer.start();
                feeding = true;
            }

            shooter.setSpindexerPower(1);
            shooter.setTransitionPower(0.6);
            shooter.setRampPower(0.7);

        }


    }

    @Override
    public boolean isFinished() {
      //  return false;
        return feeding && shootTimer.hasElapsed(5 /*TODO PICK A TIME*/);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelPower(0);
        shooter.setTurretPower(0);
        shooter.setHoodPower(0);
        shooter.stopFeeding();
    }



    private Translation2d getHubPosition() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isPresent()) {
            isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

        return isRed
                ? new Translation2d(12.286869, 4.034)
                : new Translation2d(4.624, 4.034);
    }

/*
    private Translation2d getTargetPosition(Target target) {

        boolean isRed = false;

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                isRed = true;


            }
        }

        // TODO find translations??

        if (target == Target.HUB) {

            if (isRed) {
                return new Translation2d(12.040, 4.164);
            } else {
                return new Translation2d(4.524, 4.212);
            }

        } else if (target == Target.DEPOT) {

            if (isRed) {
                return new Translation2d(16.232, 2.026);
            } else {
                return new Translation2d(0.286, 5.837);
            }

        } else if (target == Target.OUTPOST) {

            if (isRed) {
                return new Translation2d(16.383, 7.310);
            } else {
                return new Translation2d(-0.152, 0.734);
            }
        }

        return new Translation2d();


    }


 */

    private double calculateShooterSpeed(double distance) {
        return 6.18842*distance+2429.32725;
        // return shooterMap.get(distance);
    }

    private double calcHoodAngle(double distance) {
        return 0.000586636*Math.pow(distance, 2) - (0.0564512*distance) + 187.01223;

        // return hoodMap.get(distance);

    }

}