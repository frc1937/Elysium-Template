package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.poseestimation.photoncamera.CameraFactory.VISION_SIMULATION;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.MAXIMUM_AMBIGUITY;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.TAG_ID_TO_POSE;

public class AprilTagsCamera extends PhotonCameraIO {
    private final PhotonCamera photonCamera;
    private final org.photonvision.PhotonPoseEstimator photonPoseEstimator;

    public AprilTagsCamera(String cameraName, Transform3d robotCenterToCamera) {
        super(cameraName, robotCenterToCamera);

        photonCamera = new PhotonCamera(cameraName);
        photonPoseEstimator = new org.photonvision.PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotCenterToCamera
        );

        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        if (GlobalConstants.Mode.SIMULATION == CURRENT_MODE)
            VISION_SIMULATION.addCamera(photonCamera, robotCenterToCamera);
    }

    private void logVisibleTags(boolean hasResult, Optional<EstimatedRobotPose> optionalEstimatedRobotPose) {
        if (!hasResult) {
            Logger.recordOutput("UsedTags/" + photonCamera.getName(), new Pose2d[0]);
            return;
        }

        final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();
        final Pose2d[] visibleTagPoses = new Pose2d[estimatedRobotPose.targetsUsed.size()];

        for (int i = 0; i < visibleTagPoses.length; i++) {
            final int currentId = estimatedRobotPose.targetsUsed.get(i).getFiducialId();
            final Pose2d currentTagPose = TAG_ID_TO_POSE.get(currentId).toPose2d();

            visibleTagPoses[i] = currentTagPose;
        }

        Logger.recordOutput("UsedTags/" + photonCamera.getName(), visibleTagPoses);
    }

    private boolean hasResult(Optional<EstimatedRobotPose> optionalEstimatedRobotPose) {
        if (optionalEstimatedRobotPose.isEmpty())
            return false;

        final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

        if (estimatedRobotPose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) return true;

        return estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity() < MAXIMUM_AMBIGUITY;
    }

    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        double distanceSum = 0;

        for (PhotonTrackedTarget currentTarget : targets) {
            final Translation2d distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            distanceSum += distanceTranslation.getNorm();
        }

        return distanceSum / targets.size();
    }

    @Override
    protected void refreshInputs(CameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();
        final Optional<EstimatedRobotPose> optionalEstimatedRobotPose = photonPoseEstimator.update(photonCamera.getLatestResult());

        inputs.hasResult = hasResult(optionalEstimatedRobotPose);

        if (inputs.hasResult) {
            final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

            if (photonCamera.getLatestResult().getBestTarget() != null)
                Logger.recordOutput("CameraPitch/" + photonCamera.getName(), photonCamera.getLatestResult().getBestTarget().getPitch());

            inputs.estimatedRobotPose = estimatedRobotPose.estimatedPose;
            inputs.lastResultTimestamp = estimatedRobotPose.timestampSeconds;
            inputs.visibleTags = estimatedRobotPose.targetsUsed.size();
            inputs.averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
        } else {
            inputs.visibleTags = 0;
            inputs.estimatedRobotPose = new Pose3d();
        }

        logVisibleTags(inputs.hasResult, optionalEstimatedRobotPose);
    }
}
