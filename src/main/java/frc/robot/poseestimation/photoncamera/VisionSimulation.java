package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSimulation {
    private final VisionSystemSim visionSystemSim = new VisionSystemSim("main");
    private final SimCameraProperties properties = new SimCameraProperties();

    public VisionSimulation() {
        visionSystemSim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
        setupCameraProperties();
    }

    /**
     * The field where the simulated camera will be drawn.
     * @return field2d
     */
    public Field2d getDebugField() {
        return visionSystemSim.getDebugField();
    }

    public void addCamera(PhotonCamera camera, Transform3d robotCenterToCamera) {
        final PhotonCameraSim simulatedCamera = new PhotonCameraSim(camera, properties);

        visionSystemSim.addCamera(simulatedCamera, robotCenterToCamera);

        simulatedCamera.enableDrawWireframe(true);
    }

    public void updateRobotPose(Pose2d pose) {
        visionSystemSim.update(pose);
    }

    private void setupCameraProperties() {
        properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        properties.setCalibError(0.35, 0.10);
        properties.setFPS(15);
        properties.setAvgLatencyMs(50);
        properties.setLatencyStdDevMs(15);
    }
}
