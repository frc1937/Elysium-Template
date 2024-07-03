package frc.lib.generic.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.APRIL_TAG_FIELD_LAYOUT;

public class AdvancedVisionSimulation extends GenericSimulation {
    private final VisionSystemSim visionSystem = new VisionSystemSim("main");
    private final PhotonCameraSim cameraSimulation;
//todo: What the fuck is going on here... fix
    public AdvancedVisionSimulation(PhotonCamera photonCamera, Transform3d robotCenterToCamera) {
        visionSystem.addAprilTags(APRIL_TAG_FIELD_LAYOUT);

        SimCameraProperties properties = new SimCameraProperties();

        properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        properties.setCalibError(0.15, 0.050);
        properties.setFPS(20);
        properties.setAvgLatencyMs(50);
        properties.setLatencyStdDevMs(15);

        cameraSimulation = new PhotonCameraSim(photonCamera, properties);

        visionSystem.addCamera(cameraSimulation, robotCenterToCamera);

        cameraSimulation.enableRawStream(true);
        cameraSimulation.enableProcessedStream(true);
        cameraSimulation.enableDrawWireframe(true);
    }


    @Override
    public double getPositionRotations() {
        return 0;
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    void update() {
        SmartDashboard.putData("Debug-Field", visionSystem.getDebugField());

        visionSystem.update(RobotContainer.POSE_ESTIMATOR.getCurrentPose());
    }

    @Override
    void setVoltage(double voltage) {

    }
}
