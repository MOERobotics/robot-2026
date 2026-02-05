// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Converter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TankDriveForward;
import frc.robot.container.MiniBotContainer;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.TankDrive;
import frc.robot.subsystem.Vision;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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



    PhotonCamera rearCam = new PhotonCamera("PhotonCamera2");

    List<PhotonPipelineResult> result;
    //PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCamFront);
    public static final Transform3d kRobotToCamRear =
            new Transform3d(new Translation3d( Inches.of(0).in(Meter),
                    Inches.of(0).in(Meter), Inches.of(7).in(Meter)),
                    new Rotation3d(0,0, Math.PI));

    VisionSystemSim visionSim;
    PhotonCameraSim frontCameraSim;
    PhotonCameraSim rearCameraSim;
    SimCameraProperties cameraProps;
    String matchState;
    List<Pose3d> frontTargetPoses;
    List<Pose3d> rearTargetPoses;

    Vision visionSubsystem = new Vision();
   // public TankDriveForward driveForward = new TankDriveForward((TankDrive) robot.tankDrive,
    //        24, 0.1, visionSubsystem.getPose(), visionSubsystem);

    public Robot() {
    }


    @Override
    public void robotInit() {
        frontTargetPoses = new ArrayList<>();
        rearTargetPoses = new ArrayList<>();

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

        Logger.recordOutput("MatchDetails/Alliance", String.valueOf(DriverStation.getAlliance()));
        Logger.recordOutput("MatchDetails/CurrentMatchTime", DriverStation.getMatchTime());
        if (DriverStation.getMatchTime() >= 120) {
            matchState = "Auto";
        } else if (DriverStation.getMatchTime() < 120 && DriverStation.getMatchTime() >= 110) {
            matchState = "Transition";
        } else if (DriverStation.getMatchTime() < 110 && DriverStation.getMatchTime() >= 105) {
            matchState = "Shift 1";
        } else if (DriverStation.getMatchTime() < 105 && DriverStation.getMatchTime() >= 80) {
            matchState = "Shift 2";
        } else if (DriverStation.getMatchTime() < 80 && DriverStation.getMatchTime() >= 55) {
            matchState = "Shift 3";
        } else if (DriverStation.getMatchTime() < 55 && DriverStation.getMatchTime() >= 30) {
            matchState = "Shift 4";
        } else if (DriverStation.getMatchTime() < 30 && DriverStation.getMatchTime() >= 0) {
            matchState = "Endgame";
        } else {
            matchState = "Game Over";
        }
        Logger.recordOutput("MatchDetails/CurrentMatchState", matchState);
        ;

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
       // scheduler.schedule(driveForward);
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
    visionSubsystem.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        Pose2d sim2Pose = robot.getTankDrive().getPose();
        visionSubsystem.visionSim.update(sim2Pose);
    }

}
