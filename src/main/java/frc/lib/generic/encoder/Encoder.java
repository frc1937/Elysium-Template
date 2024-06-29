package frc.lib.generic.encoder;

import com.ctre.phoenix6.StatusSignal;
import frc.lib.generic.Properties;

public interface Encoder {
    /** Returns the encoder position, in Rotations*/
    double getEncoderPosition();
    double getEncoderVelocity();

    void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz);

    StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType);

    boolean configure(EncoderConfiguration encoderConfiguration);
}
