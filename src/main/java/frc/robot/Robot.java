// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;

    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);

    private Command shooterTestCommand = new ShooterTestCommand(robot,driverJoystick, functionJoystick);

    private Command shooterTeleopCommand = new ShooterTeleopCommand(robot, functionJoystick);

    public Command rotateCommand = new AutoRotateCommand(robot,driverJoystick);

    public Command driveTeleopCommand = new DriveTeleopCommand(robot,driverJoystick);


    public Command hubLoggingCommand = new HubLoggingCommand(driverJoystick);





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
        scheduler.schedule(hubLoggingCommand);
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
        scheduler.schedule(shooterTeleopCommand);
    }

    @Override
    public void teleopPeriodic() {
/*
        ChassisSpeeds robotSpeed = new ChassisSpeeds(
                MathUtil.applyDeadband(driverJoystick.getRawAxis(1) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(0) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(2) * -1, deadband));

        robot.getRobotSwerveDrive().robotDrive(robotSpeed, false);


 */

        if(driverJoystick.getRawButton(1)){
            robot.getRobotSwerveDrive().setPose(new Pose2d(robot.getRobotSwerveDrive().getPose().getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? Rotation2d.kZero : Rotation2d.kPi));
        }


        AngularVelocity rollerVelocity;

        if (functionJoystick.getRawButton(6)) {
            rollerVelocity = RPM.of(1);
        } else if (functionJoystick.getRawButton(5)) {
            rollerVelocity = RPM.of(-1);
        } else {
            rollerVelocity = RPM.of(0);
        }

        robot.getCollectorSubsystem().setRollerVelocity(rollerVelocity);



        if (driverJoystick.getPOV() != -1) {
            scheduler.cancel(driveTeleopCommand);
            scheduler.schedule(rotateCommand);

        } else {
            {
                scheduler.cancel(rotateCommand);
                scheduler.schedule(driveTeleopCommand);
            }
        }


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
