// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.container.MiniSwerve;
import frc.robot.container.MiniSwerve;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new SubMOErine();
    public Joystick driveJoystick = new Joystick(0);
    private CommandScheduler scheduler;
    // public ChassisSpeeds robotSpeed = new ChassisSpeeds(driveJoystick.getRawAxis(5), driveJoystick.getRawAxis(0), driveJoystick.getRawAxis(6));

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
                 driveJoystick.getRawAxis(5) * -1,
                 driveJoystick.getRawAxis(0),
                 driveJoystick.getRawAxis(6)
         );
        robot.getRobotSwerveDrive().robotDrive(robotSpeed);
        boolean buttonPressed;
        if (driveJoystick.getRawButton(1)) {
            robot.getClimber().setVelocity(InchesPerSecond.of(1));
        }
        else if (driveJoystick.getRawButton(2)){
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
        robot.getClimber().simulationPeriodic();
    }


}
