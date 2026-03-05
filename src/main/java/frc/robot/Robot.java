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
import frc.robot.commands.PathsFollower;
import frc.robot.commands.TankDriveForward;
import frc.robot.container.MiniBotContainer;
import frc.robot.commands.ClimberAutoCommand;
import frc.robot.commands.ClimberTeleopCommand;
import frc.robot.commands.ClimberTestCommand;
import frc.robot.commands.ShooterTestCommand;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
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

    private Command shooterTestCommand = new ShooterTestCommand(robot,driverJoystick, functionJoystick);

    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);
    public PathsFollower testPath = new PathsFollower("Curved Path");

    public ClimberTeleopCommand climberTeleopCommand = new ClimberTeleopCommand(robot, driverJoystick);

    public ClimberAutoCommand climberAutoCommand = new ClimberAutoCommand(robot, true, 1.0,true);

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
        //scheduler.schedule(climberAutoCommand);
        robot.getRobotSwerveDrive().setPose(autoCommand.getAuto().path.getStartingHolonomicPose().get());
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
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }


}
