package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

public class HoodTeleopCommand extends Command {



    public ShooterSubsystem shooterSubsystem;

    public SwerveDriveSubsystem swerveDriveSubsystem;


    public Joystick joystick;

    public PIDController hoodPID = new PIDController(0.05, 0,0);


    public Angle initalAngle = Degrees.of(0);

    public Angle finalAngle = Degrees.of(0);


    public HoodTeleopCommand (RobotContainer robot, Joystick joystick){
        shooterSubsystem = robot.getShooterSubsystem();
        swerveDriveSubsystem = robot.getRobotSwerveDrive();
        this.joystick = joystick;
        addRequirements(shooterSubsystem, swerveDriveSubsystem);
    }

    @Override
    public void initialize() {

       initalAngle = shooterSubsystem.getHoodAngleFromThroughbore();
    }

    @Override
    public void execute() {
        double output = hoodPID.calculate(shooterSubsystem.getHoodAngleFromThroughbore().in(Degree), finalAngle.in(Degrees));
        double hoodPow =0;

        if(joystick.getRawButton(1)){
            hoodPow=0.5;
        }

        shooterSubsystem.setHoodPower(output*hoodPow);



    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodPower(0);
    }


    @Override
    public boolean isFinished() {
        return hoodPID.atSetpoint();

    }




}
