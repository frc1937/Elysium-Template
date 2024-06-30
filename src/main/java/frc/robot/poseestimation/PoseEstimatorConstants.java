package frc.robot.poseestimation;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimation.photonposeestimator.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;

import java.util.HashMap;
import java.util.Map;

public class PoseEstimatorConstants {
    /**
     * The vector represents how ambiguous each value is.
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * Increase these numbers to trust the odometry less.
     */
    static final Vector<N3> ODOMETRY_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.0002);

    static final double TRANSLATION_STD_EXPONENT = 0.005;
    static final double ROTATION_STD_EXPONENT = 0.01;

    public static final double MAXIMUM_AMBIGUITY = 0.2;

    public static final Pose2d[] EMPTY_POSE_LIST = new Pose2d[0];

    public static final TargetModel TAG_MODEL = TargetModel.kAprilTag36h11;

    public static final PhotonPoseEstimator.PoseStrategy
            PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_HEADING;

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

    static {
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
    }
}
