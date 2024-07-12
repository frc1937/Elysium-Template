package frc.lib.generic.motor;

public class MotorSignal {
    public enum SignalType {
        CURRENT, POSITION, VELOCITY, VOLTAGE, TEMPERATURE, CLOSED_LOOP_TARGET
    }

    private final SignalType type;
    private final boolean useFasterThread;

    public MotorSignal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public MotorSignal(SignalType type) {
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
