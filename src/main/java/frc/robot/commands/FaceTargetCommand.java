package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.command.Command;
import frc.robot.subsystem.CameraControl;
import frc.robot.subsystem.TankDrive;
import org.littletonrobotics.junction.Logger;


public class FaceTargetCommand extends Command {
    public Angle desiredTurnAngle;
    public Angle initialAngle;
    public TankDrive driveSystem;

    public double power;

    public CameraControl camera;

    private PIDController PID;
    public FaceTargetCommand(double power, TankDrive driveSystem, CameraControl camera){
        this.power = power;
        this.camera = camera;
        PID = new PIDController(0.05, 0.0, 0.0);
        PID.setTolerance(2.0);
        PID.enableContinuousInput(-180,180);
        this.driveSystem = driveSystem;
        addRequirements(driveSystem);

    }


    @Override
    public void execute() {


        if (!camera.hasTargets()) {
            driveSystem.drive(0,0);
            return;
        }

        double yaw = camera.angleToTarget().orElse(0.0);
        SmartDashboard.putNumber("yaw", yaw);

        double currentAngle = driveSystem.getAngle().in(Units.Degrees);
        double targetAngle = currentAngle + yaw;

        double output = MathUtil.clamp(PID.calculate(yaw, 0), -1, 1);



        SmartDashboard.putNumber("curr angle", currentAngle);


        driveSystem.drive(power * output, -power * output);

        Logger.recordOutput("FaceTarget/Yaw", yaw);
        Logger.recordOutput("FaceTarget/TargetAngle", targetAngle);
        Logger.recordOutput("FaceTarget/PIDOutput", output);

        Logger.recordOutput("Power Output R", power*output);
        Logger.recordOutput("Power Output L", -power*output);


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