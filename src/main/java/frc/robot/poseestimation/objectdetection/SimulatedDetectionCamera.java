package frc.robot.poseestimation.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class SimulatedDetectionCamera extends DetectionCameraIO {
    private static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_DISTANCE_METERS = 5,
            MINIMUM_DISTANCE_METERS = 0.05;

    private static final Translation2d[] NOTES_ON_FIELD = new Translation2d[]{
            new Translation2d(2.9, 7),
            new Translation2d(2.9, 5.5),
            new Translation2d(2.9, 4.1),
            new Translation2d(8.3, 7.45),
            new Translation2d(8.3, 5.75),
            new Translation2d(8.3, 4.1),
            new Translation2d(8.3, 2.45),
            new Translation2d(8.3, 0.75),
            new Translation2d(13.65, 7),
            new Translation2d(13.65, 5.5),
            new Translation2d(13.65, 4.1)
    };

    private final String name;

    public SimulatedDetectionCamera(String name) {
        super(name);

        this.name = name;
    }

    @Override
    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) {
        final Rotation2d closestObjectYaw = getClosestVisibleObjectYaw(POSE_ESTIMATOR.getCurrentPose());

        if (closestObjectYaw != null) {
            inputs.closestTargetYaw = closestObjectYaw.getDegrees();
            inputs.yaws = new double[]{inputs.closestTargetYaw};
        }
    }

    private Rotation2d getClosestVisibleObjectYaw(Pose2d robotPose) {
        Rotation2d closestObjectYaw = null;
        double closestDistance = Double.POSITIVE_INFINITY;

        for (Translation2d objectPlacement : NOTES_ON_FIELD) {
            final Rotation2d angleToObject = getAngleFromPoseToPose(objectPlacement, robotPose.getTranslation());

            if (!isWithinHorizontalFOV(angleToObject, robotPose) || !isWithinDistance(objectPlacement, robotPose))
                continue;

            final double distance = getObjectDistance(objectPlacement, robotPose);

            if (distance < closestDistance) {
                closestObjectYaw = angleToObject.minus(robotPose.getRotation());
                closestDistance = distance;
            }
        }

        return closestObjectYaw;
    }

    private boolean isWithinHorizontalFOV(Rotation2d objectYaw, Pose2d robotPose) {
        return Math.abs(objectYaw.minus(robotPose.getRotation()).getRadians()) <= HORIZONTAL_FOV.getRadians() / 2;
    }

    private boolean isWithinDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final double distance = getObjectDistance(objectPlacement, robotPose);
        return distance <= MAXIMUM_DISTANCE_METERS && distance >= MINIMUM_DISTANCE_METERS;
    }

    private double getObjectDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d difference = objectPlacement.minus(robotPose.getTranslation());
        return difference.getNorm();
    }
}
