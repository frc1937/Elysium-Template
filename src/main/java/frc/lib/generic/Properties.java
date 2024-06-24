package frc.lib.generic;

public class Properties {

    public enum FeedforwardType {
        SIMPLE, ARM, ELEVATOR
    }

    public enum GravityType {
        ELEVATOR, ARM
    }

    public enum SignalType {
        CURRENT, POSITION, VELOCITY, VOLTAGE, TEMPERATURE, CLOSED_LOOP_TARGET
    }
}
