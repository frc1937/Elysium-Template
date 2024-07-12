package frc.lib.generic.pigeon;

import frc.lib.generic.advantagekit.LoggableHardware;

public interface Pigeon extends LoggableHardware {
    void resetConfigurations();

    double getYaw();
    double getPitch();
    double getRoll();

    void setGyroYaw(double yawDegrees);


    /** Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.*/
    void setupSignalUpdates(PigeonSignal signal);
}
