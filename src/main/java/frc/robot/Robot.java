// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FaceTargetCommand;
import frc.robot.Commands.HubLoggingCommand;
import frc.robot.container.MiniBotContainer;
import frc.robot.commands.*;
import frc.robot.commands.ClimberTestCommand;
import frc.robot.commands.ShooterTestCommand;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.CameraControl;
import frc.robot.subsystem.TankDrive;
import frc.robot.container.SubMOErine;
import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.InchesPerSecond;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;


    public Command hubLogging = new HubLoggingCommand();

    CameraControl photonCameraObject = new CameraControl(new Transform3d( new Translation3d(0,0,0), new Rotation3d(0,0,0)),"HD_Camera");


    VisionSystemSim visionSim;
    PhotonCameraSim cameraSim;
    SimCameraProperties cameraProps;

    public Command faceTargetCommand = new FaceTargetCommand(0.2, robot.getTankDrive(), photonCameraObject, 1);



    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);

    public Command shooterTestCommand = new ShooterTestCommand(robot,driverJoystick, functionJoystick);

    public Command fuelCollectorTeleopCommand = new FuelCollectorTeleopCommand(robot,functionJoystick);

    public Command rotateCommand = new frc.robot.commands.AutoRotateCommand(robot,functionJoystick);

    public Command driveTeleopCommand = new frc.robot.commands.DriveTeleopCommand(robot,driverJoystick);




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
        scheduler.schedule(hubLogging);
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
        scheduler.schedule(faceTargetCommand);
    }

    @Override
    public void teleopInit() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            robot.getRobotSwerveDrive().setPose(new Pose2d(robot.getRobotSwerveDrive().getPose().getTranslation(), Rotation2d.kPi));
        }


    }

    @Override
    public void teleopPeriodic() {


        if(driverJoystick.getRawButtonPressed(1)){
            robot.getRobotSwerveDrive().setPose(new Pose2d(robot.getRobotSwerveDrive().getPose().getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? Rotation2d.kZero:Rotation2d.kPi));
        }


        if (functionJoystick.getPOV() != -1) {

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
        scheduler.schedule(rotateCommand);

        scheduler.schedule(climberTestCommand);
        scheduler.schedule(shooterTestCommand);
        scheduler.schedule(fuelCollectorTeleopCommand);

    }

    @Override
    public void simulationInit() {
        visionSim = new VisionSystemSim("main");


        visionSim.addAprilTags(
                photonCameraObject.photonEstimator.getFieldTags()
        );
        cameraProps = new SimCameraProperties();

        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(90));
        cameraProps.setFPS(20);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(5);


        cameraSim = new PhotonCameraSim(photonCameraObject.camera,cameraProps);
        visionSim.addCamera(cameraSim,photonCameraObject.robotToCam);
    }

    @Override
    public void simulationPeriodic() {
        Pose2d sim2Pose = robot.getTankDrive().getPose();
        visionSim.update(sim2Pose);
    }


}
