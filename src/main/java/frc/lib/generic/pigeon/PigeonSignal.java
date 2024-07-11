package frc.lib.generic.pigeon;

public class PigeonSignal {
    public enum SignalType {
        YAW, PITCH, ROLL
    }

    private final SignalType type;
    private final boolean useFasterThread;

    public PigeonSignal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public PigeonSignal(SignalType type) {
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
