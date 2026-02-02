package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TankDrive;
import org.littletonrobotics.junction.Logger;

public class TankDriveForward extends Command {
    public TankDrive localDriveSystem;
    public Distance desiredDistance;
    public Distance initDistanceR;
    public Distance initDistanceL;
    public PIDController drivePID = new PIDController(0.2,0,0);
    public Pose3d currentPose;
    public Pose3d initPose;
    public Pose3d desiredPose = new Pose3d(new Translation3d(
            currentPose.getTranslation().getX() + initPose.getTranslation().getX(),
            currentPose.getTranslation().getY() + initPose.getTranslation().getY(),
            currentPose.getTranslation().getZ() + initPose.getTranslation().getZ()),
    new Rotation3d(currentPose.getRotation().getX() + initPose.getRotation().getX(),
            currentPose.getRotation().getY() + initPose.getRotation().getY(),
            currentPose.getRotation().getZ() + initPose.getRotation().getZ()
            ));
    public double localPower;
    public TankDriveForward(TankDrive driveSystem, Distance travelInFeet, double power, Pose3d initalPose, Pose3d currentPose1){
        desiredDistance = travelInFeet;
        localDriveSystem = driveSystem;
        localPower = power;
        initPose = initalPose;
        currentPose = currentPose1;
        addRequirements(localDriveSystem);
    }

    @Override
    public void initialize() {
        initDistanceR = Units.Inches.of(localDriveSystem.motorControlR.getEncoder().getPosition());
        initDistanceL = Units.Inches.of(localDriveSystem.motorControlL.getEncoder().getPosition());
        drivePID.reset();
        //drivePID.setTolerance(0.04);
    }

    @Override
    public void execute() {
        Distance endDistanceL = initDistanceL.plus(desiredDistance);
        Distance endDistanceR = initDistanceR.plus(desiredDistance);
        double pidOffsetL = MathUtil.clamp(drivePID.calculate(localDriveSystem.motorControlL.getEncoder().getPosition(), endDistanceL.in(Units.Inches)),-1,1);
        double pidOffsetR = MathUtil.clamp(drivePID.calculate(localDriveSystem.motorControlR.getEncoder().getPosition(), endDistanceR.in(Units.Inches)),-1,1);
        localDriveSystem.drive(localPower*pidOffsetR, localPower*pidOffsetL);
        Logger.recordOutput("Power Output R (w/ PID)", localPower*pidOffsetR);
        Logger.recordOutput("Power Output L (w/ PID)", localPower*pidOffsetL);
        Logger.recordOutput("End Distance R: ", endDistanceR);
    }

    @Override
    public boolean isFinished() {
        return drivePID.atSetpoint();
        /*
        boolean isForward = desiredDistance.gt(Inches.of(0));
        if (isForward &&
                localDriveSystem.ReturnEncoderTicksInches(false).minus(initDistanceR).lt(desiredDistance) &&
                localDriveSystem.ReturnEncoderTicksInches(true).minus(initDistanceL).lt(desiredDistance)){
            return false;
        } else if (!isForward &&
                localDriveSystem.ReturnEncoderTicksInches(false).minus(initDistanceR).gt(desiredDistance) &&
                localDriveSystem.ReturnEncoderTicksInches(true).minus(initDistanceL).gt(desiredDistance)){
            return false;
        } else {
            System.out.println("command ended");
            return true;
        }
         */
        //return (localDriveSystem.ReturnEncoderTicksInches(false).minus(initDistanceR).abs(Units.Inches) > desiredDistance.abs(Units.Inches));
        //Distance currentDistanceR = localDriveSystem.ReturnEncoderTicksInches(false).minus(initDistanceR);
        //return currentDistanceR.gte(desiredDistance);
    }

    @Override
    public void end(boolean interrupted) {
        localDriveSystem.drive(0,0);
        System.out.println("command ended - drive");
    }
}

