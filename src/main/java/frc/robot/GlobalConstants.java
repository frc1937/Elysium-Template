package frc.robot;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class GlobalConstants {
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
