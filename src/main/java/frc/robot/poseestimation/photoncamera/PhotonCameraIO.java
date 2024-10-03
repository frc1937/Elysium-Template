package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class PhotonCameraIO {
    private final String name;
    private final Transform3d robotCenterToCamera;

    private final CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();

    private double lastUpdatedTimestamp;

    public PhotonCameraIO(String name, Transform3d robotCenterToCamera) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }

    public int getVisibleTags() {
        return inputs.visibleTags;
    }

    public double getAverageDistanceFromTags() {
        return inputs.averageDistanceFromTags;
    }

    public boolean hasNewResult() {
        return inputs.hasResult && inputs.averageDistanceFromTags != 0 && isNewTimestamp();
    }

    public Pose2d getRobotPose() {
        return inputs.estimatedRobotPose.toPose2d();
    }

    protected void refreshInputs(CameraInputsAutoLogged inputs) { }

    public void refresh() {
        refreshInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    @AutoLog
    public static class CameraInputs {
        public boolean hasResult = false;
        public int visibleTags = 0;

        public double lastResultTimestamp = 0;
        public double averageDistanceFromTags = 0;

        public Pose3d estimatedRobotPose = new Pose3d();
    }
}
