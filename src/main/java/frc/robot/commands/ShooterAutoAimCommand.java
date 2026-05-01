package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.floor;
import static java.lang.Math.sin;

public class ShooterAutoAimCommand extends Command {

    public final ShooterSubsystem shooter;
    public final Joystick joystick;
    private final SwerveDriveSubsystem drive;


    boolean isFlywheelOn = false;

    public double kP = 1.8 / 1500.0;
    public double kI = 0.0005;
    public double kD = 0.8 / 14000;
    public double IZone = 500;

    public double hoodKP = 0.056;
    public double hoodKI = 0;
    public double hoodKD = 0;

    public double deadZone = 0.4;

    public double turretKP = 0.10;
    public double turretKI = 0;
    public double turretKD = 0;

    PIDController shooterPIDController = new PIDController(kP, kI, kD);
    PIDController hoodPIDController = new PIDController(hoodKP, hoodKI, hoodKD);
    PIDController turretPIDController = new PIDController(turretKP, turretKI, turretKD);

    public double turretSetpoint, hoodSetpoint, shooterSetpoint;

    double currDistance=0;
    double distance=0;

    double desiredTurretAngle=0;

    Translation2d turretPosition = new Translation2d();

    Translation2d hubPosition = new Translation2d();


    private Timer shootTimer = new Timer();
    boolean feeding = false;



    boolean auto = false;

    private ShooterAutoAimCommand(RobotContainer robot, Joystick joystick, boolean auto) {
        this.joystick = joystick;
        this.shooter = robot.getShooterSubsystem();
        this.drive = robot.getRobotSwerveDrive();

        shooterPIDController.setTolerance(100);
        this.auto = auto;
        addRequirements(shooter);
    }

    public ShooterAutoAimCommand(RobotContainer robot, Joystick joystick) {
        this(robot, joystick, false);
        assert joystick != null;
    }

    public ShooterAutoAimCommand(RobotContainer robot) {

        this(robot, null, true);
    }


    @Override
    public void initialize() {

        feeding = false;
        shootTimer.reset();
        shootTimer.stop();

        turretSetpoint = shooter.getSensors().turretRelativeAngle.in(Degrees);
        hoodSetpoint = shooter.getHoodAngleFromThroughbore().in(Degrees);
        shooterSetpoint = 3000;
        isFlywheelOn = false;

        turretPIDController.setTolerance(0.25);
        hoodPIDController.setSetpoint(hoodSetpoint);
        turretPIDController.setSetpoint(turretSetpoint);
        hoodPIDController.setTolerance(0.25);


        shooterPIDController.reset();
        shooterPIDController.setIZone(IZone);
        shooterPIDController.setIntegratorRange(-.10, .10);

        hubPosition = shooter.getHubPosition();


    }

    @Override
    public void execute() {
        shootOnMove();
        Pose2d pose = drive.getPose();

        Translation2d turretOffset = ShooterSubsystem.turretOffset.rotateBy(pose.getRotation());

        shooter.getSensors().turretOffset = turretOffset;
        turretPosition = shooter.getTurretPosition(pose,turretOffset);

        currDistance = shooter.getDistance(turretPosition, hubPosition).in(Inches);//- 20;


        shooter.getSensors().turretPosition = turretPosition;

        desiredTurretAngle = shooter.getTurretAimAngle(pose.getRotation(), turretPosition, hubPosition);




        distance =  shooter.getDistance(turretPosition, hubPosition).in(Inches);// - 20;


        if(auto){
            autoAim(distance);

        } else if(!auto){

            if (joystick.getRawButtonPressed(2)) {
                isFlywheelOn = !isFlywheelOn;
                shooterPIDController.reset();
            }

            if (joystick.getRawButtonPressed(1)) {
                autoAim(distance);
            }

            if (joystick.getRawButton(3)) {
                shooterSetpoint -=5;
            }
            if (joystick.getRawButton(4)) {
                shooterSetpoint += 5;
            }
            int currentPOV = joystick.getPOV();

            if (currentPOV != -1) {

                switch (currentPOV) {
                    case 90:
                        autoAim(distance);
                        break;
                }
            }



            if (joystick.getRawAxis(3) > 0.3) {
                shooter.setSpindexerPower(0.75);
            } else if (joystick.getRawAxis(2) > 0.3) {
                shooter.setSpindexerPower(-1);
                shooter.setTransitionPower(-0.6);
                shooter.setRampPower(-0.6);

            } else {
                shooter.setSpindexerPower(0);
            }

            if (joystick.getRawAxis(0) > deadZone) {
                turretSetpoint -= 0.7;
            }
            if (joystick.getRawAxis(0) < -deadZone) {
                turretSetpoint += 0.7;
            }

            if (joystick.getRawAxis(5) > deadZone) {
                hoodSetpoint -= 5.5 / 10.0;
            }
            if (joystick.getRawAxis(5) < -deadZone) {
                hoodSetpoint += 5.5 / 10.0;
            }
        }


        Logger.recordOutput("TurretPositionIThink",
                new Pose2d(
                        turretPosition,
                        new Rotation2d(pose.getRotation().plus(Rotation2d.kPi).getMeasure().plus(shooter.getSensors().turretRelativeAngle))
                )
        );


        shooter.getSensors().turretPose =  new Pose2d(
                turretPosition,
                new Rotation2d(pose.getRotation().plus(Rotation2d.kPi).getMeasure().plus(shooter.getSensors().turretRelativeAngle))
        );


        Logger.recordOutput("TargetTurretAngle",  hubPosition.minus(turretPosition).getAngle());

        Logger.recordOutput("ConstantTurretAngle", shooter.getTurretAimAngle(pose.getRotation(), turretPosition, hubPosition));



        shooter.getSensors().distanceFromHub = distance;

        shooter.getSensors().flywheelSetpoint = shooterSetpoint;
        shooter.getSensors().turretSetpoint = turretSetpoint;
        shooter.getSensors().hoodSetpoint = hoodSetpoint;


        shooter.getSensors().atShooterSpeed = shooterPIDController.atSetpoint();
        shooter.getSensors().hoodAtSetpoint = hoodPIDController.atSetpoint();
        shooter.getSensors().turretAtSetpoint = turretPIDController.atSetpoint();





        turretSetpoint = MathUtil.clamp(turretSetpoint, shooter.getSensors().turretMinAngle, shooter.getSensors().turretMaxAngle);
        turretPIDController.setSetpoint(turretSetpoint);
        double turretOutput = turretPIDController.calculate(shooter.getSensors().turretRelativeAngle.in(Degrees));
        shooter.setTurretPower(turretOutput);



        hoodSetpoint = MathUtil.clamp(hoodSetpoint, shooter.getSensors().hoodMinAngle, shooter.getSensors().hoodMaxAngle);
        hoodPIDController.setSetpoint(hoodSetpoint);
        double hoodOutput = hoodPIDController.calculate(shooter.getHoodAngleFromThroughbore().in(Degrees));
        if (hoodOutput > 0.4) hoodOutput = 0.4;
        shooter.setHoodPower(hoodOutput);




        shooterPIDController.setSetpoint(shooterSetpoint);


        if (isFlywheelOn || auto) {
            double currentRPM = shooter.getFlywheelSpeed().in(RPM);
            Logger.recordOutput("FlywheelI", shooterPIDController.getAccumulatedError() * kI);
            double output = shooterPIDController.calculate(currentRPM, shooterSetpoint);

            double feedforward = shooter.feedForwardCalc(shooterSetpoint);
            double outputMax = 1 - feedforward;

            if (output > outputMax) output = outputMax;

            shooter.setFlywheelPower(feedforward + output);
            shooter.setTransitionPower(0.7);
            shooter.setRampPower(0.6);



        } else {
            shooter.setFlywheelPower(0);
            shooter.setTransitionPower(0);
            shooter.setRampPower(0);

        }


        if (auto) {
            boolean ready = shooterPIDController.atSetpoint() &&
                            turretPIDController.atSetpoint() &&
                            hoodPIDController.atSetpoint();
            Logger.recordOutput("ready", ready);
            if (ready) {
                if (!feeding) {
                    shootTimer.reset();
                    shootTimer.start();
                    feeding = true;
                }

                shooter.setSpindexerPower(1);
            }
        }

    }


    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelPower(0);
        shooter.setRampPower(0);
        shooter.setTurretPower(0);
        shooter.setHoodPower(0);
        shooter.stopFeeding();
    }

    public void autoAim(double dist){
        hoodSetpoint = shooter.calcHoodAngle(dist);
        shooterSetpoint = shooter.calculateShooterSpeed(dist);
        turretSetpoint = MathUtil.clamp(desiredTurretAngle, shooter.getSensors().turretMinAngle, shooter.getSensors().turretMaxAngle);
    }

    public void shootOnMove(){
        double timeOfFlight = shooter.getSensors().distanceFromHub/160.0;
        ChassisSpeeds offsets = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeed().times(timeOfFlight), drive.getPose().getRotation());
        hubPosition = shooter.getHubPosition().minus(new Translation2d(offsets.vxMetersPerSecond, offsets.vyMetersPerSecond));
        Logger.recordOutput("targetPosition", new Pose2d(hubPosition, new Rotation2d(0)));
    }
    @Override
    public boolean isFinished() {
        return auto && feeding && shootTimer.hasElapsed(5);
    }

    //x, y are field coordinates of CoR
// z is field centric angle of robot
// a is distance from CoR to Center of turret (front-back)
// b is distance from CoR to CoT (left-right)
// theta = z+180 where z is robot field angle
// phi = atan2(ty-(y+a*sin(theta)-b*cos(theta),tx-(x+a*cos(theta)+b*sin(theta)) -theta


}
