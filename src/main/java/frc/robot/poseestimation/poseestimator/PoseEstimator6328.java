package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.NoSuchElementException;

import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;

public class PoseEstimator6328 {
    public record OdometryObservation(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) { }
    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) { }

    private static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

    private static PoseEstimator6328 instance;

    public static PoseEstimator6328 getInstance() {
        if (instance == null) instance = new PoseEstimator6328();
        return instance;
    }

    // Pose Estimation Members
    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    // Odometry
    private SwerveDriveWheelPositions lastWheelPositions =
            new SwerveDriveWheelPositions(
                    new SwerveModulePosition[]{
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition()
                    });

    private Rotation2d lastGyroAngle = new Rotation2d();

    private PoseEstimator6328() {
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(PoseEstimatorConstants.ODOMETRY_AMBIGUITY.get(i, 0), 2));
        }
    }

    /**
     * Add odometry observation
     */
    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = SWERVE_KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        // Check gyro connected
        if (observation.gyroAngle != null) {
            // Update dtheta for twist if gyro connected
            twist =
                    new Twist2d(
                            twist.dx, twist.dy, observation.gyroAngle.minus(lastGyroAngle).getRadians());
            lastGyroAngle = observation.gyroAngle;
        }

        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp, odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }

    public void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SECONDS
                    > observation.timestamp) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        // Get odometry based pose at timestamp
        final var sample = poseBuffer.getSample(observation.timestamp);

        if (sample.isEmpty()) return;

        // sample --> odometryPose transform and backwards of that
        final var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        final var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        final Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        final var r = new double[3];

        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs.get(i, 0) * observation.stdDevs.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        final Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());

        for (int row = 0; row < 3; ++row) {
            final double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        final Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose);
        // scale transform by visionK
        final var kTimesTransform =
                visionK.times(
                        VecBuilder.fill(
                                transform.getX(), transform.getY(), transform.getRotation().getRadians()));

        final Transform2d scaledTransform =
                new Transform2d(
                        kTimesTransform.get(0, 0),
                        kTimesTransform.get(1, 0),
                        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public Pose2d samplePose(double timestamp) {
        Pose2d sample = poseBuffer.getSample(timestamp).orElse(new Pose2d());
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample);

        return estimatedPose.plus(odometryToSampleTransform);
    }

    /**
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
     */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        lastGyroAngle = initialPose.getRotation();
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    @AutoLogOutput(key = "Poses/Robot/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    @AutoLogOutput(key = "Poses/Robot/OdometryPose")
    public Pose2d getOdometryPose() {
        return odometryPose;
    }
}
