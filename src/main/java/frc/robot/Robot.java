// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
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

import java.util.Optional;

import static edu.wpi.first.wpilibj.util.Color.*;

public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    private CommandScheduler scheduler;
    private Command collectorTestCommand = new FuelCollectorTestCommand(robot, functionJoystick);
    private Command collectorTeleopCommand = new FuelCollectorTeleopCommand(robot, functionJoystick);

    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);

    private Command shooterTestCommand = new ShooterTestCommand(robot, driverJoystick, functionJoystick);


    private Command shooterTeleopAutoAimCommand = new ShooterAutoAimCommand(robot, functionJoystick);


    public Command ledBlinkingCommandRed = new LEDBlinkingCommand(kRed, robot);
    public Command ledBlinkingCommandGreen = new LEDBlinkingCommand(kGreen, robot);
    public Command ledBlinkingCommandPink = new LEDBlinkingCommand(kPink, robot);


    public Command ledSolidColorCommandRed = new LEDColorCommand(robot, kRed);
    public Command ledSolidColorCommandGreen = new LEDColorCommand(robot, kGreen);

    public Command driveTeleopCommand = new DriveTeleopCommand(robot, driverJoystick);
    public Command controllerVibrateCommandOn = new ControllerVibrateCommandOn(driverJoystick, functionJoystick);
    public Command controllerVibrateCommandOff = new ControllerVibrateCommandOff(driverJoystick, functionJoystick);
    public Command controllerVibrateTestCommand = new ControllerVIbrateTestCommand(driverJoystick, functionJoystick);
    public Command hubLoggingCommand = new HubLoggingCommand(driverJoystick);

    public Command flywheelCalibration = new CalibrationCommand(robot);

    public SerialReader serialReader = new SerialReader();





    //AutosChooser autoCommand = new AutosChooser();
    private Autos.CommandAndPose auto;

    Autos.CommandAndPose autoCommand = new Autos.CommandAndPose(Commands.none().withName("Nothing"), new Pose2d());

    // Toggle for setting our pose to starting pose while disabled
    public boolean autoSetpoint = true;




    @Override
    public void robotInit() {

        serialReader.start();
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
        Logger.recordOutput("constantDistance", robot.getShooterSubsystem().getDistance(robot.getShooterSubsystem().getSensors().turretPosition, robot.getShooterSubsystem().getHubPosition()));

        if (serialReader.incomingData.peek() != null) {
            StringBuilder piData = new StringBuilder();
            piData.append("New data: <<<<");
            String newData;
            while ((newData = serialReader.incomingData.poll()) != null) {
                piData.append(newData);
            }
            piData.append(">>>>");
            Logger.recordOutput("piData", piData.toString());
        }
        if (autoCommand != null) {
            Logger.recordOutput("command", autoCommand.command().getName());

        }

        if (autoCommand != null) Logger.recordOutput("AutoCommand Pose", autoCommand.pose());
        Logger.recordOutput("AutoSetpoint", autoSetpoint);

    }

    @Override
    public void disabledInit() {

        scheduler.cancelAll();


    }

    @Override
    public void disabledPeriodic() {
        autoCommand = Autos.getSelectedAuto();
        if (driverJoystick.getRawButtonPressed(3)) {
            setFieldPose();
        }

        robot.getPdh().setSwitchableChannel(!functionJoystick.getRawButton(1));


    }
        @Override
        public void autonomousInit () {

            setFieldPose();

            scheduler.schedule(autoCommand.command());



        /*

                   scheduler.schedule(flywheelCalibration);


            scheduler.schedule(Commands.sequence(
                new FlywheelRatesCommand(robot,0.4),
                new FlywheelRatesCommand(robot,0.5),
                new FlywheelRatesCommand(robot,0.6),
                new FlywheelRatesCommand(robot,0.7),
                new FlywheelRatesCommand(robot,0.8),
                new FlywheelRatesCommand(robot,0.9)
            ));

             */


        }

        @Override
        public void autonomousPeriodic () {

        /*
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingHolonomicPose().get());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());
        Logger.recordOutput("idfk lmao", testPath.path.getPathPoses().toArray(Pose2d[]::new));

         */

        }

        @Override
        public void teleopInit () {



            scheduler.schedule(climberTestCommand);

            scheduler.schedule(driveTeleopCommand);


            scheduler.schedule(shooterTeleopAutoAimCommand);

            scheduler.schedule(collectorTeleopCommand);
            // scheduler.schedule(climberTeleopCommand);

        }

        @Override
        public void teleopPeriodic () {
            boolean teamAllianceWonR = DriverStation.getGameSpecificMessage().equals("R") && DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
            boolean teamAllianceWonB = DriverStation.getGameSpecificMessage().equals("B") && DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
            boolean teamAllianceWon = teamAllianceWonR || teamAllianceWonB;
            Logger.recordOutput("teamAllianceWon", teamAllianceWon);

        if (!teamAllianceWon) {
            if (DriverStation.getMatchTime() <= 140/*2:20*/ && DriverStation.getMatchTime() > 110/*1:50*/) {
                scheduler.schedule(ledSolidColorCommandGreen);
            } else if (DriverStation.getMatchTime() <= 110/*1:50*/ && DriverStation.getMatchTime() >= 105/*1:45*/) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 105/*1:45*/ && DriverStation.getMatchTime() > 85/*1:25*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 85/*1:25*/ && DriverStation.getMatchTime() >= 80/*1:20*/) {
                activeTransition();
            } else if (DriverStation.getMatchTime() < 80/*1:20*/ && DriverStation.getMatchTime() > 60/*1:00*/) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 60/*1:00*/ && DriverStation.getMatchTime() >= 55)/*55*/ {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 55/*55*/ && DriverStation.getMatchTime() > 35/*35*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 35 && DriverStation.getMatchTime() >= 30) {
                activeTransition();
            } else {
                scheduler.cancel(controllerVibrateCommandOn);
                scheduler.cancel(controllerVibrateCommandOff);
                scheduler.cancel(ledBlinkingCommandGreen);
                scheduler.cancel(ledBlinkingCommandRed);
                scheduler.cancel(ledSolidColorCommandRed);
                scheduler.schedule(ledSolidColorCommandGreen);

            }
        }

        if (teamAllianceWon) {
            if (DriverStation.getMatchTime() <= 140/*2:20*/ && DriverStation.getMatchTime() > 135)/*2:15*/ {
                scheduler.schedule(ledSolidColorCommandGreen);
            } else if (DriverStation.getMatchTime() <= 135/*2:15*/ && DriverStation.getMatchTime() >= 130/*2:10*/) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 130/*2:10*/ && DriverStation.getMatchTime() > 110/*1:50*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 110/*1:50*/ && DriverStation.getMatchTime() >= 105/*1:45*/) {
                activeTransition();
            } else if (DriverStation.getMatchTime() < 105/*1:45*/ && DriverStation.getMatchTime() > 85/*1:25*/) {
                activePeriod();
            } else if (DriverStation.getMatchTime() <= 85/*1:25*/ && DriverStation.getMatchTime() >= 80/*1:20*/) {
                inactiveTransition();
            } else if (DriverStation.getMatchTime() < 80/*1:20*/ && DriverStation.getMatchTime() > 60/*1:00*/) {
                inactivePeriod();
            } else if (DriverStation.getMatchTime() <= 60 && DriverStation.getMatchTime() >= 55) {
                activeTransition();
            } else {
                scheduler.cancel(controllerVibrateCommandOn);
                scheduler.cancel(controllerVibrateCommandOff);
                scheduler.cancel(ledBlinkingCommandGreen);
                scheduler.cancel(ledBlinkingCommandRed);
                scheduler.schedule(ledSolidColorCommandGreen);


            }





            }

            if (driverJoystick.getRawButtonPressed(1)) {
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


            if(robot.getShooterSubsystem().getSensors().atShooterSpeed){
                scheduler.schedule(ledBlinkingCommandPink);
            }else{
                scheduler.cancel(ledBlinkingCommandPink);
            }


        }
    public void inactiveTransition() {
        scheduler.cancel(ledSolidColorCommandGreen);// active to inactive
        scheduler.schedule(controllerVibrateCommandOff, ledBlinkingCommandGreen);

    }

    public void activeTransition() {
        scheduler.cancel(ledSolidColorCommandRed); // inactive to active
        scheduler.schedule(controllerVibrateCommandOn, ledBlinkingCommandRed);
    }

    public void inactivePeriod() {
        scheduler.cancel(ledBlinkingCommandGreen);
        scheduler.cancel(controllerVibrateCommandOff);
        scheduler.schedule(ledSolidColorCommandRed);

    }

    public void activePeriod() {
        scheduler.cancel(ledBlinkingCommandRed);
        scheduler.cancel(controllerVibrateCommandOn);
        scheduler.schedule(ledSolidColorCommandGreen);

    }

        @Override
        public void testInit () {
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
        public void simulationInit () {
        }

        @Override
        public void simulationPeriodic () {
        }


        public void setFieldPose () {
            if(autoCommand != null) {
                Pose2d startingPoseBlue = autoCommand.pose();
                final Pose2d startingPose;
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                    startingPose = FlippingUtil.flipFieldPose(startingPoseBlue);
                } else {
                    startingPose = startingPoseBlue;
                }
                robot.getRobotSwerveDrive().setPose(startingPose);
                Logger.recordOutput("RobotInitPose", startingPose);

            }

        }


    }



