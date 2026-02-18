package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.ShooterControl;
import frc.robot.subsystem.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class ShooterTestCommand extends Command {
    ShooterSubsystem shooterSubsystem;


    Joystick driverJoystick;

    Joystick functionJoystick;

    // driver joystick
    int transitionFW = 5;

    int transitionBack = 6;

    int spindexerFW = 7;

    int spindexerBack = 8;


    // function joystick

    int hoodUp = 7;

    int hoodDown = 5;

    int turretLeft = 8;

    int turretRight = 6;

    int flywheelFW = 4;

    int flywheelBack = 3;


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
        if (driverJoystick.getRawButton(transitionFW)) {
            transitionPower =0.2;
        } else if (driverJoystick.getRawButton(transitionBack)) {
            transitionPower=-0.2;
        }

        shooterSubsystem.setTransitionPower(transitionPower);

        double hoodPower =0;
        if (functionJoystick.getRawButton(hoodUp)) {
            hoodPower=0.2;
        } else if (functionJoystick.getRawButton(hoodDown)) {
            hoodPower=-0.2;
        }
        shooterSubsystem.setHoodPower(hoodPower);

        double turretPower =0;
        if (functionJoystick.getRawButton(turretLeft)) {
           turretPower=0.2;
        } else if (functionJoystick.getRawButton(turretRight)) {
            turretPower=-0.2;
        }

        shooterSubsystem.setTurretPower(turretPower);

        double spindexerPower=0;
        if (driverJoystick.getRawButton(spindexerFW)) {
           spindexerPower =0.2;
        } else if (driverJoystick.getRawButton(spindexerBack)) {
            spindexerPower=-0.2;
        }

        shooterSubsystem.setSpindexerPower(spindexerPower);

        double flywheelPower = 0;

        if (functionJoystick.getRawButton(flywheelFW)) {
           flywheelPower=0.2;
        } else if (functionJoystick.getRawButton(flywheelBack)) {
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