// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Converter;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.container.MiniBotContainer;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import frc.robot.subsystem.Collector;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.InchesPerSecond;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;
    private Command collectorTestCommand = new FuelCollectorTestCommand(robot, functionJoystick);
    private Command collectorTeleopCommand = new FuelCollectorTeleopCommand(robot, functionJoystick);

    private Command shooterTestCommand = new ShooterTestCommand(robot,driverJoystick, functionJoystick);
    public Command climberTestCommand = new ClimberTestCommand(robot, driverJoystick);
    public PathsFollower testPath = new PathsFollower("Curved Path");

    public Command climberTeleopCommand = new ClimberTeleopCommand(robot, driverJoystick);

    public Command climberAutoCommand = new ClimberAutoCommand(robot, true, 1.0,true);

    public Command collectorAutoCommand = new FuelCollectorAutoCommand(robot, false, false, "out");

    AutosChooser autoCommand = new AutosChooser();

    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
        MOELogger.setupLogging(this);
        scheduler = CommandScheduler.getInstance();
        scheduler.schedule(FollowPathCommand.warmupCommand());
    }


    @Override
    public void driverStationConnected() {
        AutosChooser.setupAutos(robot);
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
       // scheduler.schedule(collectorAutoCommand);
        //scheduler.schedule(climberAutoCommand);

        robot.getRobotSwerveDrive().setPose( new PathsFollower("ALT-Depot").path.getStartingHolonomicPose().get());
        scheduler.schedule(autoCommand.getAuto());
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingHolonomicPose().get());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());
/*
        robot.getTankDrive().setPose(testPath.path.getStartingDifferentialPose());
        scheduler.schedule(testPath);
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingDifferentialPose());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());

 */
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        scheduler.schedule(collectorTeleopCommand);
        scheduler.schedule(climberTeleopCommand);
    }

    @Override
    public void teleopPeriodic() {
        ChassisSpeeds robotSpeed = new ChassisSpeeds(
                MathUtil.applyDeadband(driverJoystick.getRawAxis(1) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(0) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(2) * -1, deadband)
        );
      ;
        robot.getRobotSwerveDrive().robotDrive(robotSpeed);

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        scheduler.schedule(climberTestCommand);
        scheduler.schedule(shooterTestCommand);
        scheduler.schedule(collectorTestCommand);
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }


}
