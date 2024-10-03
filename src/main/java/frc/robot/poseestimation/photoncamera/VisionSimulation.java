package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class VisionSimulation {
    private final VisionSystemSim visionSystemSim = new VisionSystemSim("main");
    private final SimCameraProperties properties = new SimCameraProperties();

    public VisionSimulation() {
        visionSystemSim.addAprilTags( AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
        setupCameraProperties();
    }

    public void addCamera(PhotonCamera camera, Transform3d robotCenterToCamera) {
        final PhotonCameraSim simulatedCamera = new PhotonCameraSim(camera, properties);

        visionSystemSim.addCamera(simulatedCamera, robotCenterToCamera);

        simulatedCamera.enableDrawWireframe(true);
    }

    public void updateCurrentPose() {
        visionSystemSim.update(POSE_ESTIMATOR.getCurrentPose());
    }

    private void setupCameraProperties() {
        properties.setCalibration(960, 720, Rotation2d.fromDegrees(70));
        properties.setCalibError(0, 0);
        properties.setFPS(30);
        properties.setAvgLatencyMs(0);
        properties.setLatencyStdDevMs(0);
    }
}
