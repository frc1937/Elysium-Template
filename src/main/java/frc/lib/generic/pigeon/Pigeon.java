package frc.lib.generic.pigeon;

import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.AutoLog;

public class Pigeon implements LoggableHardware {
    public void resetConfigurations() {
    }

    public double getYaw() {
        return 0;
    }

    public double getPitch() {
        return 0;
    }

    public double getRoll() {
        return 0;
    }

    public void setGyroYaw(double yawDegrees) {
    }

    @Override
    public void periodic() {
    }

    @Override
    public PigeonInputsAutoLogged getInputs() {
        return new PigeonInputsAutoLogged();
    }

    @Override
    public void close() {
    }

    /**
     * Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.
     */
    public void setupSignalUpdates(PigeonSignal signal) {
    }

    @AutoLog
    public static class PigeonInputs {
        public double gyroYawDegrees = 0;
        public double gyroRollDegrees = 0;
        public double gyroPitchDegrees = 0;

        public double[] odometryUpdatesTimestamp = new double[0];
        public double[] odometryUpdatesYawDegrees = new double[0];
        public double[] odometryUpdatesPitchDegrees = new double[0];
        public double[] odometryUpdatesRollDegrees = new double[0];
    }
}
