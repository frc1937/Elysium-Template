package frc.robot.poseestimation.objectdetection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonDetectionCamera extends DetectionCameraIO {
    private final PhotonCamera camera;

    public PhotonDetectionCamera(String name) {
        super(name);

        PhotonCamera.setVersionCheckEnabled(false);
        camera = new PhotonCamera(name);
    }

    @Override
    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) {
        if (camera == null || !camera.isConnected())
            return;

        final var result = camera.getLatestResult();

        if (!result.hasTargets()) return;

        inputs.yaws = result.getTargets().stream().mapToDouble((iteratedResult) -> -iteratedResult.getYaw()).toArray();
        inputs.closestTargetYaw = getClosestTargetYaw(result);
    }

    private double getClosestTargetYaw(PhotonPipelineResult result) {
        double lowestSum = 1000;
        double closestTargetYaw = 0;

        for (PhotonTrackedTarget target : result.getTargets()) {
            final double currentCameraDistance = Math.abs(target.getYaw()) + Math.abs(target.getPitch());

            if (lowestSum > currentCameraDistance) {
                lowestSum = currentCameraDistance;
                closestTargetYaw = -target.getYaw();
            }
        }

        return closestTargetYaw;
    }
}
