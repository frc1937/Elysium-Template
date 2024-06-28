package frc.robot;

import frc.lib.util.TunableNumber;

public class GlobalConstants {
    public static final boolean IS_TUNING_MODE = true;

    public static final Mode CURRENT_MODE = Mode.SIMULATION;

    public enum Mode {
        REAL, SIMULATION, REPLAY;
    }

    public static final double FIELD_LENGTH_METRES = 14.56;

}
