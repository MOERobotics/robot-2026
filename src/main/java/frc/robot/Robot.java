// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.InchesPerSecond;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;


    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);

        MOELogger.setupLogging(this);
        scheduler = CommandScheduler.getInstance();


    }


    @Override
    public void driverStationConnected() {
    }

    @Override
    public void robotPeriodic() {
        MOELogger.log();
        scheduler.run();
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        ChassisSpeeds robotSpeed = new ChassisSpeeds(
                MathUtil.applyDeadband(driverJoystick.getRawAxis(1) * -1, deadband),
                MathUtil.applyDeadband( driverJoystick.getRawAxis(0) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(2) * -1, deadband)
        );
      ;
        robot.getRobotSwerveDrive().robotDrive(robotSpeed);

        boolean buttonPressed;
        if (driverJoystick.getRawButton(1)) {
            robot.getClimber().setVelocity(InchesPerSecond.of(1));
        }
        else if (driverJoystick.getRawButton(2)){
            robot.getClimber().setVelocity(InchesPerSecond.of(-1));
        } else {
            robot.getClimber().stopVelocity();
        }


    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }


}
