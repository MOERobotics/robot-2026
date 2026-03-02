package frc.robot.container;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystem.interfaces.TankDriveSubsystem;

public class MiniBotContainer extends RobotContainer {

    private TankDriveSubsystem tankDrive;
    private PowerDistribution pdh;
    public SparkMax motorControlL = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
   // public Vision visionSubsystem;

    public SparkMax motorControlR = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);


    public MiniBotContainer() {
        System.out.println("Constructed RobotContainer type: " + getClass());
        System.out.println("Constructed RobotContainer type: " + getClass());
        //tankDrive = new TankDrive(motorControlL, motorControlR);
        //visionSubsystem = new Vision();
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        /*
        AutoBuilder.configure(
                tankDrive::getPose, // Robot pose supplier
                tankDrive::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                tankDrive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> tankDrive.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPLTVController(0), // PPLTVController is the built in path following controller for differential drive trains
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                tankDrive // Reference to this subsystem to set requirements
        );



    }
    @Override
    public void toLog(LogTable table) {
        table.put("TankDrive", tankDrive);
        table.put("rightPower", tankDrive);
    }

    @Override
    public void fromLog(LogTable table) {
        tankDrive = table.get("TankDrive", tankDrive);
    }

    public TankDrive getTankDrive(){
        return (TankDrive) tankDrive;
    }

         */

    }
}
