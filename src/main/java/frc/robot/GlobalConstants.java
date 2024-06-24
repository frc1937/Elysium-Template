package frc.robot;

public class GlobalConstants {
    public static final boolean IS_TUNING_MODE = true;

    public static final Mode CURRENT_MODE = Mode.REAL;

    public enum Mode {
        REAL, SIMULATION, REPLAY;
    }

    public static final double FIELD_LENGTH_METRES = 14.56;
}
