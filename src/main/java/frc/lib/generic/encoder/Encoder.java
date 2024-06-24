package frc.lib.generic.encoder;

import frc.lib.generic.Properties;

public interface Encoder {
    double getEncoderPosition();
    double getEncoderVelocity();

    void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz);

    boolean configure(EncoderConfiguration encoderConfiguration);
}
