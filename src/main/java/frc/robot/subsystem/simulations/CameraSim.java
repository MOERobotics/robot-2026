package frc.robot.subsystem.simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.MOESimulator;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.CameraControl;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class CameraSim implements MOESimulator {

    private final VisionSystemSim visionSim;
    private final RobotContainer robot;


    public CameraSim(CameraControl cameraControl, RobotContainer robot){


        visionSim = new VisionSystemSim("main");
        this.robot = robot;

        visionSim.addAprilTags(
                cameraControl.photonEstimator.getFieldTags()
        );
        SimCameraProperties cameraProps = new SimCameraProperties();

        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(90));
        cameraProps.setFPS(20);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(5);


        PhotonCameraSim cameraSim = new PhotonCameraSim(cameraControl.camera, cameraProps);
        visionSim.addCamera(cameraSim,cameraControl.robotToCam);
    }
    @Override
    public void updateSimState() {
        Pose2d sim2Pose = robot.getRobotSwerveDrive().getPose();
        visionSim.update(sim2Pose);
    }

    @Override
    public void simulationPeriodic() {

    }
}
