package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterTestCommand extends Command {
    ShooterSubsystem shooterSubsystem;
    Joystick driverJoystick;

    Joystick functionJoystick;

    // driver joystick
    int transitionFWBtn = 5;

    int transitionBackBtn = 6;

    int spindexerFWBtn = 7;

    int spindexerBackBtn = 8;


    // function joystick

    int hoodUpBtn = 7;

    int hoodDownBtn = 5;

    int turretLeftBtn = 8;

    int turretRightBtn = 6;

    int flywheelFWBtn = 4;

    int flywheelBackBtn = 3;


    public ShooterTestCommand(RobotContainer robot, Joystick driverjoystick, Joystick functionJoystick) {
        this.shooterSubsystem = robot.getShooterSubsystem();
        this.driverJoystick = driverjoystick;
        this.functionJoystick = functionJoystick;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
        double transitionPower =0;
        if (driverJoystick.getRawButton(transitionFWBtn)) {
            transitionPower =0.2;
        } else if (driverJoystick.getRawButton(transitionBackBtn)) {
           transitionPower=-0.2;
        }

        shooterSubsystem.setTransitionPower(transitionPower);

        double hoodPower =0;
        if (functionJoystick.getRawButton(hoodUpBtn)) {
            hoodPower=0.2;
        } else if (functionJoystick.getRawButton(hoodDownBtn)) {
            hoodPower=-0.2;
        }
        shooterSubsystem.setHoodPower(hoodPower);

        double turretPower =0;
        if (functionJoystick.getRawButton(turretLeftBtn)) {
           turretPower=0.2;
        } else if (functionJoystick.getRawButton(turretRightBtn)) {
            turretPower=-0.2;
        }

        shooterSubsystem.setTurretPower(turretPower);

        double spindexerPower=0;
        if (driverJoystick.getRawButton(spindexerFWBtn)) {
           spindexerPower =0.2;
        } else if (driverJoystick.getRawButton(spindexerBackBtn)) {
            spindexerPower=-0.2;
        }

        shooterSubsystem.setSpindexerPower(spindexerPower);

        double flywheelPower = 0;

        if (functionJoystick.getRawButton(flywheelFWBtn)) {
           flywheelPower=0.2;
        } else if (functionJoystick.getRawButton(flywheelBackBtn)) {
            flywheelPower=-0.2;
        }

        shooterSubsystem.setFlywheelPower(flywheelPower);

        Logger.recordOutput("reachedHoodMax", shooterSubsystem.reachedHoodMax());
        Logger.recordOutput("reachedHoodMin", shooterSubsystem.reachedHoodMin());
        Logger.recordOutput("reachedTurretMax", shooterSubsystem.reachedTurretMax());
        Logger.recordOutput("reachedTurretMin", shooterSubsystem.reachedTurretMin());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelPower(0);
        shooterSubsystem.setHoodPower(0);
        shooterSubsystem.setTurretPower(0);
        shooterSubsystem.stopFeeding();
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}