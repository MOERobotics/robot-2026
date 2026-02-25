package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.ShooterSubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class ShooterTeleopCommand extends  Command {

        public ShooterSubsystem shooterSubsystem;
        public SwerveDriveSubsystem swerveDriveSubsystem;
        public Joystick joystick;
        double flywheelPower;
        AngularVelocity targetFlywheelPower;
        boolean isFlywheelOn = false;
        public double kP = 1/500.0;
        public double kI = 0;
        public double kD = 0;
        private static final double targetRPM = 3000;
        PIDController shooterPIDController = new PIDController(kP,kI,kD);

        public ShooterTeleopCommand(Joystick joystick, double flywheelPower) {
          this.joystick = joystick;
          this.flywheelPower = flywheelPower;

          shooterPIDController.setSetpoint(targetRPM);
          shooterPIDController.setTolerance(100); // Dont know if this is needed but a fine safety net ig

          addRequirements(shooterSubsystem);
        }


        @Override
        public void initialize() {
        //    shooterPIDController.reset(); isnt needed right now since kI and kD are zero
        }

        @Override
        public void execute() {
            // Flywheel Toggle
            if (joystick.getRawButtonPressed(2)) {
                isFlywheelOn = !isFlywheelOn;

                if (isFlywheelOn) {
                    AngularVelocity currentRPM = shooterSubsystem.getFlywheelSpeed();
                    double output = shooterPIDController.calculate(currentRPM.in(RPM));
                    shooterSubsystem.setFlywheelPower(shooterPIDController.calculate(output));
                } else {
                    shooterSubsystem.setFlywheelPower(0);
                }
            }
            // Shoot Button
            if (joystick.getRawButtonPressed(12)){
               if(isFlywheelOn && shooterPIDController.atSetpoint()) {
                   shooterSubsystem.setSpindexerPower(0.2);
                   shooterSubsystem.setTransitionPower(0.2);
               } else {
                   shooterSubsystem.stopFeeding();
               }
            } else {
                shooterSubsystem.stopFeeding();
            }

        }

        public void end(boolean interrupted) {
            shooterSubsystem.setFlywheelPower(0);
            shooterSubsystem.setTurretPower(0);
            shooterSubsystem.stopFeeding();

        }



        @Override
        public boolean isFinished() {
            return false;
        }

    }


