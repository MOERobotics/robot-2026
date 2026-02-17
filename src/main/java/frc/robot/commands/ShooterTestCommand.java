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

        if (driverJoystick.getRawButton(transitionFW)) {
            shooterSubsystem.setTransitionPower(0.2);
        } else if (driverJoystick.getRawButtonPressed(transitionBack)) {
            shooterSubsystem.setTransitionPower(-0.2);
        }

        if (functionJoystick.getRawButton(hoodUp)) {
            shooterSubsystem.setHoodPower(0.2);
        } else if (functionJoystick.getRawButtonPressed(hoodDown)) {
            shooterSubsystem.setHoodPower(-0.2);
        }

        if (functionJoystick.getRawButton(turretLeft)) {
            shooterSubsystem.setTurretPower(0.2);
        } else if (functionJoystick.getRawButtonPressed(turretRight)) {
            shooterSubsystem.setTurretPower(-0.2);
        }

        if (driverJoystick.getRawButton(spindexerFW)) {
            shooterSubsystem.setSpindexerPower(0.2);
        } else if (driverJoystick.getRawButtonPressed(spindexerBack)) {
            shooterSubsystem.setSpindexerPower(-0.2);
        }


        if (functionJoystick.getRawButton(flywheelFW)) {
            shooterSubsystem.setFlywheelPower(0.2);
        } else if (functionJoystick.getRawButtonPressed(flywheelBack)) {
            shooterSubsystem.setFlywheelPower(-0.2);
        }


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
        return shooterSubsystem.reachedHoodMin() ||
                shooterSubsystem.reachedHoodMax() ||
                shooterSubsystem.reachedTurretMin() ||
                shooterSubsystem.reachedTurretMax();


    }

}