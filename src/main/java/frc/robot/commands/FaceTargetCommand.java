package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.CameraControl;
import frc.robot.subsystem.TankDrive;
import frc.robot.subsystem.interfaces.PhotonCameraSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

public class FaceTargetCommand extends Command {

    private final SwerveDriveSubsystem driveSystem;
    private final PhotonCameraSubsystem camera;
    private final PIDController PID;
    private final ChassisSpeeds speeds;

    int target;

    Angle goalYaw;

    public FaceTargetCommand(ChassisSpeeds speeds, RobotContainer robot, int target) {

        this.speeds = speeds;
        this.driveSystem = robot.getRobotSwerveDrive();
        this.camera = robot.getPhotonCamera();

        this.PID = new PIDController(0.4, 0.0, 0);
        PID.setTolerance(1.5);
        PID.enableContinuousInput(-180, 180);
        this.target = target;

        addRequirements(driveSystem);
    }


    @Override
    public void initialize(){
        PID.reset();

        if(camera.angleToTarget(target).isPresent()){
            goalYaw = driveSystem.getPose().getRotation().getMeasure().plus(Degrees.of(camera.angleToTarget(target).get().getDegrees()));
        } else {
            goalYaw = Degrees.of(0);
        }

    }
    public void execute() {

        if (!camera.hasTargets()) {
            driveSystem.robotDrive(speeds,false);
        }




        double output = PID.calculate(driveSystem.getPose().getRotation().getMeasure().in(Degrees), goalYaw.in(Degrees));
        Logger.recordOutput("face orientation output", output);
        driveSystem.robotDrive(new ChassisSpeeds(0,0, output * speeds.omegaRadiansPerSecond), false);

        Logger.recordOutput("FaceTarget/Yaw", goalYaw);
        Logger.recordOutput("FaceTarget/Output", output);
        Logger.recordOutput("FaceTarget/Error", PID.getAccumulatedError());
    }

    @Override
    public boolean isFinished() {
        return PID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSystem.robotDrive(new ChassisSpeeds(0,0,0),false);
    }
}
