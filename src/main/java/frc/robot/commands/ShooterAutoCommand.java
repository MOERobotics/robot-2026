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

    private PIDController flywheelPID = new PIDController(1 / 200.0, 0.0001, 0.1 / 17500);
    private PIDController turretPID = new PIDController(0.0055, 0, 0);
    private PIDController hoodPID = new PIDController(0.056, 0, 0);
    private Timer shootTimer = new Timer();
    boolean feeding = false;


    private double flywheelRPM = 4300;

    public enum Target {
        HUB,
        DEPOT,
        OUTPOST
    }


    public ShooterAutoCommand(RobotContainer robot, Target target) {

        this.shooter = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();
        this.selectedTarget = target;

        flywheelPID.setTolerance(300);
        turretPID.setTolerance(1.0);
        hoodPID.setTolerance(1.0);




        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        targetPosition = getTargetPosition(selectedTarget);

        shootTimer.reset();
        shootTimer.stop();


        feeding = false;
    }

    @Override
    public void execute() {


        Pose2d pose = drive.getPose();


        double flywheelOutput = flywheelPID.calculate(shooter.getFlywheelSpeed().in(RPM), flywheelRPM);

        flywheelOutput = MathUtil.clamp(flywheelOutput, 0, 0.55);


        Logger.recordOutput("flywheelOutput", flywheelOutput);

        shooter.setFlywheelPower(0.45 + flywheelOutput);




        Rotation2d targetTurretAngle = targetPosition.minus(pose.getTranslation()).getAngle();




        Rotation2d robotHeading = pose.getRotation();

        Rotation2d desiredTurret = targetTurretAngle.minus(robotHeading);


        double currTurretAngle = shooter.getTurretAngleinDegrees().in(Degrees);

        double desiredAngle = desiredTurret.getDegrees();

        desiredAngle = MathUtil.inputModulus(desiredAngle, 0, 360);


        double desTurretAngle = MathUtil.clamp(desiredAngle, 30,330);

        double turretOutput = turretPID.calculate(currTurretAngle, desTurretAngle);




       if( turretOutput >0.3) {
           turretOutput=0.3;
        }




        shooter.setTurretPower(turretOutput);

        Logger.recordOutput("turretOutput", turretOutput);
        Logger.recordOutput("turretDesiredAngle", desiredTurret.getDegrees());
        Logger.recordOutput("turretCurrentAngle", targetTurretAngle.getDegrees());


        double distance = pose.getTranslation().getDistance(targetPosition);


        double currHoodAngle = shooter.getHoodAngleFromThroughbore().in(Degrees);

        double desHoodAngle = MathUtil.clamp(calculateHoodAngle(distance), 180,220);


        double hoodOutput = hoodPID.calculate(currHoodAngle,desHoodAngle);

        hoodOutput = MathUtil.clamp(hoodOutput, -0.32, 0.32);



        shooter.setHoodPower(hoodOutput);


        boolean flywheelReady = flywheelPID.atSetpoint();


        boolean turretReady = turretPID.atSetpoint();


        boolean hoodReady = hoodPID.atSetpoint();


        Logger.recordOutput("hoodReady", flywheelReady);
        Logger.recordOutput("turretReady", turretReady);
        Logger.recordOutput("hoodReady", hoodReady);


        if (flywheelReady && turretReady && hoodReady) {

            if (!feeding) {
                shootTimer.start();
                feeding = true;
            }

            shooter.setSpindexerPower(1);
            shooter.setTransitionPower(0.7);
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


    private double calculateHoodAngle(double distance) {

        //TODO implement this with linear regression????
        return 190;
    }


}