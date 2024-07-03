package frc.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.*;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.EMPTY_POSE_LIST;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged inputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final RobotPoseSourceIO robotPoseSourceIO;
    private double lastUpdatedTimestamp;
    private Pose2d cachedPose = null;

    public RobotPoseSource(String name, Transform3d robotCenterToCamera) {
        this.name = name;
        this.robotCenterToCamera = new Transform3d();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            robotPoseSourceIO = new PhotonCameraIO(name, robotCenterToCamera);
        else
            robotPoseSourceIO = new RobotPoseSourceIO();
    }

    public static double[] pose3dToDoubleArray(Pose3d pose) {
        if (pose == null)
            return new double[0];

        return new double[]{
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                pose.getRotation().getX(),
                pose.getRotation().getY(),
                pose.getRotation().getZ()
        };
    }

    public void update() {
        robotPoseSourceIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);
        cachedPose = getUnCachedRobotPose();
        if (!inputs.hasResult || inputs.averageDistanceFromTags == 0 || cachedPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", EMPTY_POSE_LIST);
        else
            Logger.recordOutput("Poses/Robot/" + name + "Pose", cachedPose);
    }

    public int getVisibleTags() {
        return inputs.visibleTags;
    }

    public double getAverageDistanceFromTags() {
        return inputs.averageDistanceFromTags;
    }

    public boolean hasNewResult() {
        return (inputs.hasResult && inputs.averageDistanceFromTags != 0) && isNewTimestamp();
    }

    public Pose2d getRobotPose() {
        return cachedPose;
    }

    public String getName() {
        return name;
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }

    private Pose2d getUnCachedRobotPose() {
        final Pose3d cameraPose = doubleArrayToPose3d(inputs.cameraPose);
        if (cameraPose == null)
            return null;

        return cameraPose.transformBy(robotCenterToCamera.inverse()).toPose2d();
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private Pose3d doubleArrayToPose3d(double[] doubleArray) {
        if (doubleArray == null || doubleArray.length != 6)
            return null;

        return new Pose3d(
                new Translation3d(doubleArray[0], doubleArray[1], doubleArray[2]),
                new Rotation3d(doubleArray[3], doubleArray[4], doubleArray[5])
        );
    }
}
