package frc.lib.generic.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.lib.generic.Properties;

public interface Encoder {
    /** Returns the encoder position, in Rotations*/
    double getEncoderPosition();
    double getEncoderVelocity();

    void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz);

    StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType);

    /**
     * Refreshes all status signals.
     * This has the same effect as calling {@link com.ctre.phoenix6.BaseStatusSignal#refreshAll(BaseStatusSignal...)}}}.
     * DO NOT USE if not necessary.
     */
    void refreshStatusSignals(Properties.SignalType... signalTypes);

    boolean configure(EncoderConfiguration encoderConfiguration);
}
