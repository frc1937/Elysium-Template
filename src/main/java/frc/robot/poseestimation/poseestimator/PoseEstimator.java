package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Optimizations;
import frc.robot.GlobalConstants;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.photoncamera.PhotonCameraIO;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.poseestimation.photoncamera.CameraFactory.VISION_SIMULATION;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();

    private final PhotonCameraIO[] robotPoseSources;

    private final PoseEstimator6328 poseEstimator6328 = PoseEstimator6328.getInstance();

    /**
     * Constructs a new PoseEstimator.
     *
     * @param robotPoseSources the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator(PhotonCameraIO... robotPoseSources) {
        this.robotPoseSources = robotPoseSources;

        putAprilTagsOnFieldWidget();

        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(pose -> {
            field.getObject("path").setPoses(pose);
            Logger.recordOutput("Path", pose.toArray(new Pose2d[0]));
        });
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        updateFromVision();
        field.setRobotPose(getCurrentPose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, relative to the blue alliance's driver station right corner
     */
    public void resetPose(Pose2d currentPose) {
        RobotContainer.SWERVE.setGyroHeading(currentPose.getRotation());
        poseEstimator6328.resetPose(currentPose);
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    public Pose2d getCurrentPose() {
        return poseEstimator6328.getEstimatedPose();
    }

    public Pose2d getOdometryPose() {
        return poseEstimator6328.getOdometryPose();
    }

    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the swerve wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void addOdometryObservations(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        if (Optimizations.isColliding()) {
            DriverStation.reportWarning("The robot collided! Discarding odometry at timestamp", false);
            return;
        }

        for (int i = 0; i < swerveWheelPositions.length; i++) {
            if (swerveWheelPositions[i] == null) continue;

            poseEstimator6328.addOdometryObservation(new PoseEstimator6328.OdometryObservation(
                    swerveWheelPositions[i],
                    gyroRotations[i],
                    timestamps[i])
            );
        }
    }

    private void updateFromVision() {
        getViableVisionObservations().stream()
                .sorted(Comparator.comparingDouble(PoseEstimator6328.VisionObservation::timestamp))
                .forEach(poseEstimator6328::addVisionObservation);
    }

    private List<PoseEstimator6328.VisionObservation> getViableVisionObservations() {
        List<PoseEstimator6328.VisionObservation> viableVisionObservations = new ArrayList<>();

        for (PhotonCameraIO robotPoseSource : robotPoseSources) {
            final PoseEstimator6328.VisionObservation visionObservation = getVisionObservation(robotPoseSource);

            if (visionObservation != null)
                viableVisionObservations.add(visionObservation);

            if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
                if (visionObservation != null && visionObservation.visionPose() != null)
                    VISION_SIMULATION.getDebugField()
                        .getObject("VisionEstimation")
                        .setPose(visionObservation.visionPose());
                else {
                    VISION_SIMULATION.getDebugField().getObject("VisionEstimation").setPoses();
                }
            }
        }

        return viableVisionObservations;
    }

    private PoseEstimator6328.VisionObservation getVisionObservation(PhotonCameraIO robotPoseSource) {
        robotPoseSource.refresh();

        if (!robotPoseSource.hasNewResult())
            return null;

        final Pose2d robotPose = robotPoseSource.getRobotPose();

        if (robotPose == null)
            return null;

        return new PoseEstimator6328.VisionObservation(
                robotPose,
                robotPoseSource.getLastResultTimestamp(),
                averageDistanceToStdDevs(robotPoseSource.getAverageDistanceFromTags(), robotPoseSource.getVisibleTags())
        );
    }

    private Matrix<N3, N1> averageDistanceToStdDevs(double averageDistance, int visibleTags) {
        final double translationStd = TRANSLATION_STD_EXPONENT * Math.pow(averageDistance, 2) / (visibleTags * visibleTags);
        final double thetaStd = ROTATION_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    private void putAprilTagsOnFieldWidget() {
        for (Map.Entry<Integer, Pose3d> entry : TAG_ID_TO_POSE.entrySet()) {
            field.getObject("Tag " + entry.getKey()).setPose(entry.getValue().toPose2d());
        }
    }
}
