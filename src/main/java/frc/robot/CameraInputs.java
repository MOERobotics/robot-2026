package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

@AutoLog
public class CameraInputs {
    public PhotonPipelineResult photon1 = new PhotonPipelineResult();
    public PhotonPipelineResult photon2 = new PhotonPipelineResult();
    public Pose3d turretCamPose;
    public Pose3d swerveCamPose;

}
