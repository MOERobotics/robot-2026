package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;

import java.util.Timer;

import static edu.wpi.first.units.Units.*;

public class TurretAutoCommand extends Command {
    boolean isTurretAligned;
    boolean isScoring;
    AngularVelocity flywheelSpeed = RPM.of(0);
    Angle TurretHorizontalAngle = Degrees.of(0);
    public ShooterSubsystem shooterSubsystem;
    public SwerveDriveSubsystem swerveDriveSubsystem;

    public TurretAutoCommand(boolean isTurretAligned, boolean isScoring, AngularVelocity flywheelSpeed) {
        this.isScoring = isScoring;
        this.isTurretAligned = isTurretAligned;
    }


    @Override
    public void initialize() {
        AngularVelocity flywheelSpeed = shooterSubsystem.setFlywheelPower();
        




    }

    @Override
    public void execute() {

        shooterSubsystem.setTurretPower(0);
        shooterSubsystem.getTurretAngleinDegrees();


    }

    public void end(boolean interrupted) {
        shooterSubsystem.setTurretPower(0);
    }

    @Override
    public boolean isFinished() {
        return isTurretAligned = shooterSubsystem.reachedTurretMax();
    }

}
