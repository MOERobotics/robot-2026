package frc.robot.commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;

public class PathsFollower extends Command {
    String pathName;
    public final Command pathCommand;
    public PathPlannerPath path;

    public PathsFollower(String pathName) {
        this.pathName = pathName;
        try{
        path = PathPlannerPath.fromPathFile(pathName);

            pathCommand = AutoBuilder.followPath(
                    path
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
        addRequirements(pathCommand.getRequirements());
    }

    @Override
    public void initialize() {
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        Logger.recordOutput("PathPlannerStuff/scheduled", pathCommand.isScheduled());
        Logger.recordOutput("PathPlannerStuff/current path", pathCommand.getName());
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {

        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}