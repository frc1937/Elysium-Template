package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.poseestimation.photoncamera.CameraFactory;
import frc.robot.poseestimation.photoncamera.PhotonCameraIO;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;

import java.util.HashMap;
import java.util.Map;

public class PoseEstimatorConstants {
    static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
            0.27, 0.37, 0.19,
            new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(30))
    );

    public static final PhotonCameraIO FRONT_CAMERA = CameraFactory.generateCamera("FrontLeft1937", ROBOT_TO_FRONT_CAMERA);

    /**
     * The vector represents how ambiguous each value is.
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * Increase these numbers to trust the odometry less.
     */
    static final Vector<N3> ODOMETRY_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.001);

    static final double TRANSLATION_STD_EXPONENT = 0.005;
    static final double ROTATION_STD_EXPONENT = 0.01;

    public static final double MAXIMUM_AMBIGUITY = 0.2;

    public static final TargetModel TAG_MODEL = TargetModel.kAprilTag36h11;

    public static final PhotonPoseEstimator.PoseStrategy
            PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

    static {
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
    }
}

