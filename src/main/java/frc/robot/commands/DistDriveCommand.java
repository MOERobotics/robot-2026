package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

public class DistDriveCommand extends Command {
    RobotContainer robot;
    Distance targetDist;
    Distance currDist;
    ChassisSpeeds setChassisSpeeds;
    Translation2d initTranslation;
    Translation2d currTranslation;
    SwerveModuleState[] defaultState;


    public DistDriveCommand (RobotContainer robot, Distance targetDist, ChassisSpeeds setChassisSpeeds){
        this.robot = robot;
        this.targetDist = targetDist;
        this.setChassisSpeeds = setChassisSpeeds;
    }

    @Override
    public void initialize() {
        currDist = Meters.zero();
        initTranslation = robot.getRobotSwerveDrive().getPose().getTranslation();

        //robot.getRobotSwerveDrive().setModuleStates(defaultState);
    }

    @Override
    public void execute() {

        robot.getRobotSwerveDrive().robotDrive(setChassisSpeeds, true);
        currTranslation = robot.getRobotSwerveDrive().getPose().getTranslation();
        currDist = Meters.of(initTranslation.getDistance(currTranslation));

        Logger.recordOutput("DistDriveCommandStuff/targetDist", targetDist);
        Logger.recordOutput("DistDriveCommandStuff/currDist", currDist);
        Logger.recordOutput("DistDriveCommandStuff/currTranslation", currTranslation);
        Logger.recordOutput("DistDriveCommandStuff/initTranslation", initTranslation);

    }

    @Override
    public void end(boolean interrupted) {
        robot.getRobotSwerveDrive().stop();
    }

    @Override
    public boolean isFinished() {
        return currDist.gte(targetDist);
    }

    public void rotateModules (){
        for (int i = 0; i < defaultState.length; i++) {
            SwerveModuleState moduleState = defaultState[i];
            //moduleState.optimize(swerveModules[i].getTravelDistanceNRobotAngle().angle);

            robot.getRobotSwerveDrive().getModules()[i].setPivot(moduleState.angle);
        }
    }
}
