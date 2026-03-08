// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.fasterxml.jackson.databind.util.Converter;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.commands.autos.DepotRun;
import frc.robot.commands.autos.DepotRunTest;
import frc.robot.commands.autos.HubandBumpAutos;
import frc.robot.container.ProMOEtheus;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;


public class Robot extends LoggedRobot {

    public RobotContainer robot = new ProMOEtheus();
    public Joystick driverJoystick = new Joystick(0);
    public Joystick functionJoystick = new Joystick(1);

    public double deadband = 0.06; // find deadband number;
    private CommandScheduler scheduler;
    private Command collectorTestCommand = new FuelCollectorTestCommand(robot, functionJoystick);
    private Command collectorTeleopCommand = new FuelCollectorTeleopCommand(robot, functionJoystick);

    public ClimberTestCommand climberTestCommand = new ClimberTestCommand(robot, driverJoystick);

    private Command shooterTestCommand = new ShooterTestCommand(robot,driverJoystick, functionJoystick);

    private Command shooterTeleopCommand = new ShooterTeleopCommand(robot, functionJoystick);

    public Command rotateCommand = new AutoRotateCommand(robot,driverJoystick);

    public Command driveTeleopCommand = new DriveTeleopCommand(robot,driverJoystick);


    public Command hubLoggingCommand = new HubLoggingCommand(driverJoystick);
    public Command autoRotate = new AutoRotateCommand(robot, driverJoystick);



    public Command autoShootercommand = new ShooterAutoCommand(robot, ShooterAutoCommand.Target.HUB);

    public PathsFollower testPath = new PathsFollower("Curved Path");

    public Command climberTeleopCommand = new ClimberTeleopCommand(robot, driverJoystick);

    public Command climberAutoCommand = new ClimberAutoCommand(robot, true, 1.0,false);

    public Command collectorAutoCommand = new FuelCollectorAutoCommand(robot, true, false, "out");

    //AutosChooser autoCommand = new AutosChooser();
    private Autos.CommandAndPose auto;

    Autos.CommandAndPose autoCommand = new Autos.CommandAndPose(Commands.none(),new Pose2d());


    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
        MOELogger.setupLogging(this);
        scheduler = CommandScheduler.getInstance();
//        scheduler.schedule(FollowPathCommand.warmupCommand());
        scheduler.schedule(hubLoggingCommand);

   // auto = DepotRunTest.getAuto(robot);

    auto = HubandBumpAutos.H_LC(robot);

    }


    @Override
    public void driverStationConnected() {
      //  AutosChooser.setupAutos(robot);
    }

    @Override
    public void robotPeriodic() {
        MOELogger.log();
        scheduler.run();
       // Logger.recordOutput("command", autoCommand.getAuto().getName());

        if (auto != null) {
            Logger.recordOutput("command", auto.command().getName());

        }


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
  //      scheduler.schedule(autoShootercommand);

       // scheduler.schedule(collectorAutoCommand);
        //scheduler.schedule(climberAutoCommand);
       // testPath =  new PathsFollower("ALT-Depot");
       // robot.getRobotSwerveDrive().setPose( testPath.path.getStartingHolonomicPose().get());
        //scheduler.schedule(autoCommand.getAuto());
/*
        robot.getTankDrive().setPose(testPath.path.getStartingDifferentialPose());
        scheduler.schedule(testPath);
        Logger.recordOutput("Auto Start Pose", testPath.path.getStartingDifferentialPose());

        Logger.recordOutput("Auto End Pose", testPath.path.getGoalEndState());

 */

        /*
        autoCommand = Autos.getSelectedAuto();
        scheduler.schedule(autoCommand);

         */



        robot.getRobotSwerveDrive().setPose(auto.pose());
        scheduler.schedule(auto.command());
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
        scheduler.schedule(shooterTeleopCommand);
        scheduler.schedule(collectorTeleopCommand);
        scheduler.schedule(climberTeleopCommand);

    }

    @Override
    public void teleopPeriodic() {
/*
        ChassisSpeeds robotSpeed = new ChassisSpeeds(
       /* ChassisSpeeds robotSpeed = new ChassisSpeeds(
                MathUtil.applyDeadband(driverJoystick.getRawAxis(1) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(0) * -1, deadband),
                MathUtil.applyDeadband(driverJoystick.getRawAxis(2) * -1, deadband));

        robot.getRobotSwerveDrive().robotDrive(robotSpeed, false);


 */

        if(driverJoystick.getRawButton(1)){
            robot.getRobotSwerveDrive().setPose(new Pose2d(robot.getRobotSwerveDrive().getPose().getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? Rotation2d.kZero : Rotation2d.kPi));
        }
        /*

        AngularVelocity rollerVelocity;

        if (functionJoystick.getRawButton(6)) {
            rollerVelocity = RPM.of(0.75);
        } else if (functionJoystick.getRawButton(5)) {
            rollerVelocity = RPM.of(-0.75);
        } else {
            rollerVelocity = RPM.of(0);
        }

        AngularVelocity armVelocity;

        if (functionJoystick.getRawButton(7)) {
            armVelocity = RPM.of(0.5);
        } else if (functionJoystick.getRawButton(8)) {
            armVelocity = RPM.of(-0.25);
        } else {
            armVelocity = RPM.of(0);
        }

        robot.getCollector().setRollerVelocity(rollerVelocity);

        robot.getCollector().setArmVelocity(armVelocity);



        */


        if (driverJoystick.getPOV() != -1) {
            scheduler.cancel(driveTeleopCommand);
            scheduler.schedule(rotateCommand);

        } else {
            {
                scheduler.cancel(autoRotate);
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
        scheduler.schedule(collectorTestCommand);
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }


}
