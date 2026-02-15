package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CameraControl;
import frc.robot.subsystem.TankDrive;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

public class FaceTargetCommand extends Command {

    private final TankDrive driveSystem;
    private final CameraControl camera;
    private final PIDController PID;
    private final double power;

    int target;

    Angle goalYaw;

    public FaceTargetCommand(double power, TankDrive driveSystem, CameraControl camera, int target) {

        this.power = power;
        this.driveSystem = driveSystem;
        this.camera = camera;

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
            goalYaw = driveSystem.getAngle().plus(Degrees.of(camera.angleToTarget(target).get().getDegrees()));
        } else {
            goalYaw = Degrees.of(0);
        }

    }
    public void execute() {

        if (!camera.hasTargets()) {
            driveSystem.drive(0,0);
        }




        double output = MathUtil.clamp(PID.calculate(driveSystem.getAngle().in(Degrees)  , goalYaw.in(Degrees)), -1.0, 1.0);

        driveSystem.drive(power * output, -power * output);

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
        driveSystem.drive(0,0);
    }
}
