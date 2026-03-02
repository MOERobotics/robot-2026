package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TankDrive;

public class TankDriveForward extends Command {
    public TankDrive localDriveSystem;
    public double desiredDistance;
    public PIDController drivePID = new PIDController(0.2,0,0);
    public Pose3d finalPose;
    public Pose3d initPose;
    //public Vision localVisionSubsystem;

    public double localPower;
    public TankDriveForward(TankDrive driveSystem, double travelInInches, double power, Pose3d inputPose){
        desiredDistance = travelInInches;
        localDriveSystem = driveSystem;
        localPower = power;
        initPose = inputPose;
        finalPose = new Pose3d(new Translation3d(2+initPose.getTranslation().getX(),//+Units.Inches.of(desiredDistance).in(Units.Meters),
                initPose.getTranslation().getY(),
                initPose.getTranslation().getZ()),
                initPose.getRotation()
                );
      //  localVisionSubsystem = visionSubsystem;
        addRequirements(localDriveSystem);

    }

    @Override
    public void initialize() {

        drivePID.reset();
        //drivePID.setTolerance(0.04);
    }

    @Override
    public void execute() {
        localDriveSystem.drive(localPower, localPower);
        /*
        double pidOffsetL = MathUtil.clamp(drivePID.calculate(localDriveSystem.motorControlL.getEncoder().getPosition(), endDistanceL.in(Units.Inches)),-1,1);
        double pidOffsetR = MathUtil.clamp(drivePID.calculate(localDriveSystem.motorControlR.getEncoder().getPosition(), endDistanceR.in(Units.Inches)),-1,1);
        localDriveSystem.drive(localPower*pidOffsetR, localPower*pidOffsetL);
        Logger.recordOutput("Power Output R (w/ PID)", localPower*pidOffsetR);
        Logger.recordOutput("Power Output L (w/ PID)", localPower*pidOffsetL);
        Logger.recordOutput("End Distance R: ", endDistanceR);

         */
    }

    @Override
    public boolean isFinished() {
        //return localVisionSubsystem.getPose().getTranslation().getX() == finalPose.getTranslation().getX();
        //if ()
        //return drivePID.atSetpoint();
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
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        localDriveSystem.drive(0,0);
        System.out.println("command ended - drive");
    }
}

