package frc.robot.poseestimation.objectdetection;

import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class DetectionCameraFactory {
    public static DetectionCameraIO createDetectionCamera(String name) {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
            return new PhotonDetectionCamera(name);
        }

        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new DetectionCameraIO(name);
        }

        return new SimulatedDetectionCamera(name);
    }
}
