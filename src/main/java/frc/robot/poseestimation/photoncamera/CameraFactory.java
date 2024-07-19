package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class CameraFactory {
    public static PhotonCameraIO generateCamera(String cameraName, Transform3d robotCenterToCamera) {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL)
            return new Camera(cameraName, robotCenterToCamera);

        return new PhotonCameraIO(cameraName, robotCenterToCamera);
    }
}
