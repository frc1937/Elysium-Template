package frc.lib.generic.pigeon;

import frc.lib.generic.advantagekit.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Pigeon implements LoggableHardware {
    private final PigeonInputsAutoLogged inputs = new PigeonInputsAutoLogged();
    private final String name;

    public Pigeon(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public void resetConfigurations() {}
    public double getYaw() {return 0;}
    public double getPitch() {return 0;}
    public double getRoll() {return 0;}
    public void setGyroYaw(double yawDegrees) {}
    /**
     * Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.
     */
    public void setupSignalUpdates(PigeonSignal signal) {}

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public PigeonInputsAutoLogged getInputs() {
        return inputs;
    }
    
    protected void refreshInputs(PigeonInputsAutoLogged inputs) {}

    @AutoLog
    public static class PigeonInputs {
        public double gyroYawDegrees = 0;
        public double gyroRollDegrees = 0;
        public double gyroPitchDegrees = 0;

        public double[] timestamps = new double[0];
        public double[] threadGyroYawDegrees = new double[0];
        public double[] threadGyroPitchDegrees = new double[0];
        public double[] threadGyroRollDegrees = new double[0];
    }
}
