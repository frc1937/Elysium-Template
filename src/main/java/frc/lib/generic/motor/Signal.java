package frc.lib.generic.motor;

public class Signal {
    public enum SignalType {
        CURRENT, POSITION, VELOCITY, VOLTAGE, TEMPERATURE, CLOSED_LOOP_TARGET
    }

    private final SignalType type;
    private final boolean useFasterThread;

    public Signal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public Signal(SignalType type) {
        this(type, false);
    }

    public SignalType getType() {
        return type;
    }

    public String getName() {
        return type.name();
    }

    public double getUpdateRate() {
        return useFasterThread() ? 200 : 50;
    }

    public boolean useFasterThread() {
        return useFasterThread;
    }
}
