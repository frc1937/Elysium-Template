package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class GlobalConstants {
    public static final double ROBOT_PERIODIC_LOOP_TIME = 0.02;

    public static final Lock FASTER_THREAD_LOCK = new ReentrantLock();
    public static final double ODOMETRY_FREQUENCY_HERTZ = 200.0;

    public static final boolean IS_TUNING_MODE = true;

    public static final double GRAVITY = 9.80665;
    public static final double MINIMUM_ACCELERATION_FOR_COLLISION = 25;

    public static final double FIELD_LENGTH_METRES = 14.56;

    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public static final Pose3d RED_SPEAKER = new Pose3d(new Translation3d(-0.0381+14.56,5.547868, 1.8), new Rotation3d());
    public static final Pose3d BLUE_SPEAKER = new Pose3d(new Translation3d(-0.0381,5.547868, 2), new Rotation3d());

    public static final boolean IS_REPLAY = false;

    //for when the RoboRio doesn't have enough space...
    public static final boolean SHOULD_WRITE_LOGS = false;
    public static final boolean SHOULD_DISPLAY_MECHANISMS = true; //For when we want to save resources.

    public static final Mode CURRENT_MODE;

    public enum Mode {
        REAL, SIMULATION, REPLAY
    }

    static {
        if (Robot.isReal()) {
            CURRENT_MODE = Mode.REAL;
        } else if (Robot.isSimulation() && !IS_REPLAY) {
            CURRENT_MODE = Mode.SIMULATION;
        } else {
            CURRENT_MODE = Mode.REPLAY;
        }
    }
}
