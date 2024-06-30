package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.poseestimation.PoseEstimatorConstants.TAG_ID_TO_POSE;

public class GlobalConstants {
    public static final Pose3d BLUE_SPEAKER = TAG_ID_TO_POSE.get(7).plus(new Transform3d(new Translation3d(0.2, 0.0, 1.3), new Rotation3d()));

    public static final double ROBOT_PERIODIC_LOOP_TIME = 0.02;

    public static final double ODOMETRY_FREQUENCY_HERTZ = 200.0;
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();

    public static final boolean IS_TUNING_MODE = true;
    public static final Mode CURRENT_MODE = Mode.SIMULATION;

    public enum Mode {
        REAL, SIMULATION, REPLAY;
    }

    public static final double FIELD_LENGTH_METRES = 14.56;

    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

}
