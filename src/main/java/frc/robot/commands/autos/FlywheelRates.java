package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autos;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;

public class FlywheelRates {

    public static Autos.CommandAndPose percent10inc(RobotContainer robot) {

        Command autoCommand =  Commands.sequence(
                new FlywheelRatesCommand(robot, 0.1),
                new FlywheelRatesCommand(robot, 0.2),
                new FlywheelRatesCommand(robot, 0.3),
                new FlywheelRatesCommand(robot, 0.4),
                new FlywheelRatesCommand(robot, 0.5),
                new FlywheelRatesCommand(robot, 0.6),
                new FlywheelRatesCommand(robot, 0.7),
                new FlywheelRatesCommand(robot, 0.8),
                new FlywheelRatesCommand(robot, 0.9),
                new FlywheelRatesCommand(robot, 1)
        );



        return new Autos.CommandAndPose(autoCommand, robot.getRobotSwerveDrive().getPose());
    }
}