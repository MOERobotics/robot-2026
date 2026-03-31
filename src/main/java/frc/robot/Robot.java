// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import frc.robot.subsystem.interfaces.LEDSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/*
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 */

import java.sql.Driver;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static edu.wpi.first.wpilibj.util.Color.kRed;

public class Robot extends LoggedRobot {

    public RobotContainer robot = new SubMOErine();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;
    private Command collectorTestCommand = new FuelCollectorTestCommand(robot, functionJoystick);
    private Command collectorTeleopCommand = new FuelCollectorTeleopCommand(robot, functionJoystick);

    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);

    private Command shooterTestCommand = new ShooterTestCommand(robot, driverJoystick, functionJoystick);

    private Command shooterTeleopCommand = new ShooterTeleopCommand(robot, functionJoystick);

    private Command shooterTeleopAutoAimCommand = new ShooterTeleopAutoAimCommand(robot, functionJoystick);


    public Command rotateCommand = new AutoRotateCommand(robot, driverJoystick);
    public Command ledBlinkingCommandRed = new LEDBlinkingCommand(kRed, robot);
    public Command ledBlinkingCommandGreen = new LEDBlinkingCommand(kGreen, robot);
    public Command ledSolidColorCommandRed = new LEDColorCommand(robot, kRed);
    public Command ledSolidColorCommandGreen = new LEDColorCommand(robot, kGreen);
    public Command driveTeleopCommand = new DriveTeleopCommand(robot, driverJoystick);
    public Command controllerVibrateCommandOn = new ControllerVibrateCommandOn(driverJoystick, functionJoystick);
    public Command controllerVibrateCommandOff = new ControllerVibrateCommandOff(driverJoystick, functionJoystick);
    public Command controllerVibrateTestCommand = new ControllerVIbrateTestCommand(driverJoystick, functionJoystick);
    public Command hubLoggingCommand = new HubLoggingCommand(driverJoystick);
    public PathsFollower testPath = new PathsFollower("Curved Path");

    public Command climberTeleopCommand = new ClimberTeleopCommand(robot, driverJoystick);

    public Command climberAutoCommand = new ClimberAutoCommand(robot, true, 1.0, false);

    public Command collectorAutoCommand = new FuelCollectorAutoCommand(robot, true, false, "out");

    //AutosChooser autoCommand = new AutosChooser();
    private Autos.CommandAndPose auto;

    Autos.CommandAndPose autoCommand = new Autos.CommandAndPose(Commands.none(), new Pose2d());

    // Toggle for setting our pose to starting pose while disabled
    public boolean autoSetpoint = true;


    // public AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout();

    //  public Transform3d robotToCam = new Transform3d(0,0,0 ,new Rotation3d(0,0,0));
    //  public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,robotToCam );

    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
        MOELogger.setupLogging(this);
        scheduler = CommandScheduler.getInstance();
//        scheduler.schedule(FollowPathCommand.warmupCommand());
        scheduler.schedule(hubLoggingCommand);
        // auto = DepotRunTest.getAuto(robot);

        //  auto = OutpostAutos.outpost(robot);

    }


    @Override
    public void driverStationConnected() {
        Autos.setupAutos(robot);


    }


    @Override
    public void robotPeriodic() {
        MOELogger.log();
        scheduler.run();

        if (autoCommand != null) {
            Logger.recordOutput("command", autoCommand.command().getName());

        }

        if (autoCommand != null) Logger.recordOutput("AutoCommand Pose", autoCommand.pose());
        Logger.recordOutput("AutoSetpoint", autoSetpoint);

        robot.getRobotSwerveDrive().photonPoses();
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
    }

    @Override
    public void disabledPeriodic() {

        if (driverJoystick.getRawButtonPressed(3)) {
            autoSetpoint = !autoSetpoint;
        }
        if (autoSetpoint) {
            setFieldPose();
        }
    }

    @Override
    public void autonomousInit() {
/*
        robot.getTankDrive().setPose(testPath.path.getStartingDifferentialPose());
        scheduler.schedule(testPath);
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingDifferentialPose());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());

 */


        autoCommand = Autos.getSelectedAuto();


        setFieldPose();

        scheduler.schedule(autoCommand.command());




/*


        robot.getRobotSwerveDrive().setPose(auto.pose());
        scheduler.schedule(auto.command());


 */


    }

    @Override
    public void autonomousPeriodic() {

        /*
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingHolonomicPose().get());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());
        Logger.recordOutput("idfk lmao", testPath.path.getPathPoses().toArray(Pose2d[]::new));

         */

    }

    @Override
    public void teleopInit() {

        scheduler.schedule(climberTestCommand);

        scheduler.schedule(driveTeleopCommand);

        //scheduler.schedule(shooterTeleopCommand);

        scheduler.schedule(shooterTeleopAutoAimCommand);

        scheduler.schedule(collectorTeleopCommand);
        // scheduler.schedule(climberTeleopCommand);

    }

    @Override
    public void teleopPeriodic() {


        boolean teamAllianceWonR = DriverStation.getGameSpecificMessage().equals("R") && DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
        boolean teamAllianceWonB = DriverStation.getGameSpecificMessage().equals("B") && DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
        boolean teamAllianceWon = teamAllianceWonR || teamAllianceWonB;
        Logger.recordOutput("teamAllianceWon", teamAllianceWon);

        if (!teamAllianceWon) {
            if (DriverStation.getMatchTime() <= 140 && DriverStation.getMatchTime() >= 111) {
                scheduler.schedule(ledSolidColorCommandGreen);
            } else if (DriverStation.getMatchTime() <= 110 && DriverStation.getMatchTime() >= 105) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() <= 104 && DriverStation.getMatchTime() >= 86) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 85 && DriverStation.getMatchTime() >= 80) {
                activeTransition();
            } else if (DriverStation.getMatchTime() <= 79 && DriverStation.getMatchTime() >= 51) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 50 && DriverStation.getMatchTime() >= 45) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() <= 44 && DriverStation.getMatchTime() >= 26) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 25 && DriverStation.getMatchTime() >= 20) {
                activeTransition();
            } else {
                scheduler.schedule(ledSolidColorCommandGreen);
                scheduler.cancel(controllerVibrateCommandOn);
                scheduler.cancel(controllerVibrateCommandOff);
                scheduler.cancel(ledBlinkingCommandGreen);
                scheduler.cancel(ledBlinkingCommandRed);
                scheduler.cancel(ledSolidColorCommandRed);
            }
        }

        if (teamAllianceWon) {
            if (DriverStation.getMatchTime() <= 140 && DriverStation.getMatchTime() >= 136) {
                scheduler.schedule(ledSolidColorCommandGreen);
            } else if (DriverStation.getMatchTime() <= 135 && DriverStation.getMatchTime() >= 130) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() <= 129 && DriverStation.getMatchTime() >= 111) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 110 && DriverStation.getMatchTime() >= 105) {
                activeTransition();
            } else if (DriverStation.getMatchTime() <= 104 && DriverStation.getMatchTime() >= 86) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 85 && DriverStation.getMatchTime() >= 80) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() <= 79 && DriverStation.getMatchTime() >= 51) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 50 && DriverStation.getMatchTime() >= 45) {
                activeTransition();
            } else {
                scheduler.schedule(ledBlinkingCommandGreen);
                scheduler.cancel(controllerVibrateCommandOn);
                scheduler.cancel(controllerVibrateCommandOff);
                scheduler.cancel(ledBlinkingCommandGreen);
                scheduler.cancel(ledBlinkingCommandRed);

            }


            if (driverJoystick.getRawButton(1)) {
                robot.getRobotSwerveDrive().setPose(
                        new Pose2d(
                                robot.getRobotSwerveDrive().getPose().getTranslation(),
                                DriverStation.getAlliance()
                                        .orElse(DriverStation.Alliance.Blue) ==
                                        DriverStation.Alliance.Blue ?
                                        Rotation2d.kZero : Rotation2d.kPi
                        )
                );
            }

        }


    }


    public void inactiveTransition() {
        scheduler.cancel(ledSolidColorCommandGreen);
        scheduler.schedule(controllerVibrateCommandOff, ledBlinkingCommandRed);

    }

    public void activeTransition() {
        scheduler.cancel(ledSolidColorCommandRed);
        scheduler.schedule(controllerVibrateCommandOn, ledBlinkingCommandGreen);
    }

    public void inactivePeriod() {
        scheduler.cancel(ledBlinkingCommandRed);
        scheduler.cancel(controllerVibrateCommandOff);
        scheduler.schedule(ledSolidColorCommandRed);

    }

    public void activePeriod() {
        scheduler.cancel(ledBlinkingCommandGreen);
        scheduler.cancel(controllerVibrateCommandOn);
        scheduler.schedule(ledSolidColorCommandGreen);

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        scheduler.schedule(ledBlinkingCommandRed);
        scheduler.schedule(controllerVibrateTestCommand);
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


    public void setFieldPose() {
        assert autoCommand != null;
        Pose2d startingPoseBlue = autoCommand.pose();
        final Pose2d startingPose;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            startingPose = FlippingUtil.flipFieldPose(startingPoseBlue);
        } else {
            startingPose = startingPoseBlue;
        }
        robot.getRobotSwerveDrive().setPose(startingPose);


    }


}



