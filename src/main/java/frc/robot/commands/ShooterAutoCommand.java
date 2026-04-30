package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
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


    // copied from teleop stuff


    public double kP = 2.0 / 1500.0;
    public double kI = 0.00015;
    public double kD = 0.4 / 14000;
    private PIDController flywheelPID = new PIDController(kP, kI, kD);
    private PIDController turretPID = new PIDController(0.055, 0, 0);
    private PIDController hoodPID = new PIDController(0.056, 0, 0);
    private Timer shootTimer = new Timer();
    boolean feeding = false;

    private double flywheelRPM = 3510;


    public Translation2d hubPosition = new Translation2d();
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
        shootTimer.start();
        feeding = false;
        hubPosition = shooter.getHubPosition();
    }

    @Override
    public void execute() {


        Pose2d pose = drive.getPose();

        Translation2d turretOffset = ShooterSubsystem.turretOffset.rotateBy(pose.getRotation());


        Translation2d turretPosition = shooter.getTurretPosition(pose,turretOffset);

        shooter.getSensors().turretOffset = turretOffset;
        shooter.getSensors().turretPosition = turretPosition;


        Distance distance = shooter.getDistance(turretPosition,hubPosition);



        double flywheelSetpoint = shooter.calculateShooterSpeed(distance.in(Inches));


        flywheelPID.setSetpoint(flywheelSetpoint);

        double flywheelOutput = flywheelPID.calculate(shooter.getFlywheelSpeed().in(RPM));

        double feedforward = shooter.feedForwardCalc(flywheelSetpoint);
        double outputMax = 1 - feedforward;

        if (flywheelOutput > outputMax) flywheelOutput = outputMax;
        if (flywheelOutput < 0) flywheelOutput = 0;

        Logger.recordOutput("flywheelOutputAuto", flywheelOutput);

        Logger.recordOutput("flywheelSetpointAuto", flywheelSetpoint);


        shooter.getSensors().atShooterSpeed = flywheelPID.atSetpoint();
        shooter.setTransitionPower(0.6);
        shooter.setRampPower(0.7);

        Logger.recordOutput("turretPositioninAuto",
                new Pose2d(
                        turretPosition,
                        new Rotation2d(pose.getRotation().plus(Rotation2d.kPi).getMeasure().plus(shooter.getSensors().turretRelativeAngle))
                )
        );


         shooter.setFlywheelPower(feedforward + flywheelOutput);





        Rotation2d targetTurretAngle = shooter.getHubPosition().minus(turretPosition).getAngle();



        double currTurretAngle = shooter.getSensors().turretRelativeAngle.in(Degrees);


        double desiredAngle = shooter.getTurretAimAngle(pose.getRotation(), turretPosition, hubPosition);




        double desTurretAngleClamped = MathUtil.clamp(desiredAngle, shooter.getSensors().turretMinAngle,shooter.getSensors().turretMaxAngle);

        turretPID.setSetpoint(desTurretAngleClamped);

        double turretOutput = turretPID.calculate(currTurretAngle);




       if( turretOutput >0.3) {
           turretOutput=0.3;
        }




        shooter.setTurretPower(turretOutput);


        Logger.recordOutput("turretDesiredAngleClamped", desTurretAngleClamped);
        Logger.recordOutput("turretDesiredAngleAuto", desiredAngle);
        Logger.recordOutput("turretCurrentAngleAuto", targetTurretAngle.getDegrees());




        double currHoodAngle = shooter.getHoodAngleFromThroughbore().in(Degrees);

        double desHoodAngle = MathUtil.clamp(shooter.calcHoodAngle(distance.in(Inches)), shooter.getSensors().hoodMinAngle,shooter.getSensors().hoodMaxAngle);

        Logger.recordOutput("hoodSetpointAuto", desHoodAngle);

        hoodPID.setSetpoint(desHoodAngle);

        double hoodOutput = hoodPID.calculate(currHoodAngle);

        hoodOutput = MathUtil.clamp(hoodOutput, -0.32, 0.32);


        shooter.setHoodPower(hoodOutput);


        boolean flywheelReady = flywheelPID.atSetpoint();


        boolean turretReady = turretPID.atSetpoint();


        boolean hoodReady = hoodPID.atSetpoint();


        Logger.recordOutput("flywheelReady", flywheelReady);
        Logger.recordOutput("turretReady", turretReady);
        Logger.recordOutput("hoodReady", hoodReady);

        shooter.getSensors().hoodAtSetpoint = hoodPID.atSetpoint();
        shooter.getSensors().turretAtSetpoint = turretPID.atSetpoint();





        if (flywheelReady && turretReady && hoodReady) {

            if (!feeding) {
                shootTimer.start();
                feeding = true;
            }

            shooter.setSpindexerPower(1);



        }


    }

    @Override
    public boolean isFinished() {
      //  return false;
        return feeding && shootTimer.hasElapsed(10 /*TODO PICK A TIME*/);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelPower(0);
        shooter.setTurretPower(0);
        shooter.setHoodPower(0);
        shooter.stopFeeding();
        shooter.setRampPower(0);

    }


}