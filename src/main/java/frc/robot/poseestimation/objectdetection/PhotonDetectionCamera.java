package frc.robot.poseestimation.objectdetection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

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

        final PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) return;

        final List<PhotonTrackedTarget> results = result.getTargets();
        final double[] array =  new double[results.size()];

        for(int i = 0; i < array.length; i++) {
            array[i] = results.get(i).getYaw();
        }

        inputs.yaws = array;

        final double[] targetYawValues = getClosestTargetYawValues(result);

        inputs.closestTargetYaw = targetYawValues[0];
        inputs.closestTargetPitch = targetYawValues[1];
    }

    private double[] getClosestTargetYawValues(PhotonPipelineResult result) {
        double lowestSum = 1000;
        double closestTargetYaw = 0;
        double closestTargetPitch = 0;

        for (PhotonTrackedTarget target : result.getTargets()) {
            final double currentCameraDistance = Math.abs(target.getYaw()) + Math.abs(target.getPitch());

            if (lowestSum > currentCameraDistance) {
                lowestSum = currentCameraDistance;
                closestTargetYaw = target.getYaw();
                closestTargetPitch = target.getPitch();
            }
        }

        return new double[]{closestTargetYaw, closestTargetPitch};
    }
}
