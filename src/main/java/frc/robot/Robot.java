// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.HubLoggingCommand;
import frc.robot.container.MiniBotContainer;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.PhotonCameraObject;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new MiniBotContainer();
    public Joystick driveJoystick = new Joystick(0);
    private CommandScheduler scheduler;


    public Command hubLogging = new HubLoggingCommand();









    PhotonCameraObject photonCameraObject = new PhotonCameraObject("HD_Camera");
    List<PhotonPipelineResult> result;


    VisionSystemSim visionSim;
    PhotonCameraSim cameraSim;
    SimCameraProperties cameraProps;


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
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        double joystickX= driveJoystick.getX();
        double joystickY = -driveJoystick.getY();
        double leftPow = joystickX+joystickY;
        double rightPow = joystickY-joystickX;
        robot.getTankDrive().drive(leftPow,rightPow);

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
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
        visionSim.addCamera(cameraSim,photonCameraObject.kRobotToCam);
    }

    @Override
    public void simulationPeriodic() {
        Pose2d sim2Pose = robot.getTankDrive().getPose();
        visionSim.update(sim2Pose);
    }


}
