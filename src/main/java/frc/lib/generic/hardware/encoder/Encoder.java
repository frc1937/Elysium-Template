package frc.lib.generic.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Encoder implements LoggableHardware {
    private final EncoderInputs inputs = new EncoderInputs();
    private final String name;

    public Encoder(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    /** This is required for sim to function correctly. In real, this won't do anything. */
    public void setSimulatedEncoderPositionSource(DoubleSupplier positionSource) {}
    /** This is required for sim to function correctly. In real, this won't do anything. */
    public void setSimulatedEncoderVelocitySource(DoubleSupplier velocitySource) {}

    /** Returns the encoder position, in Rotations*/
    public double getEncoderPosition() {return inputs.position; }
    public double getEncoderVelocity() {return inputs.velocity; }

    /** Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.*/
    public void setSignalUpdateFrequency(EncoderSignal signal) {}

    public StatusSignal<Double> getRawStatusSignal(EncoderSignal signal) { return null; }

    /**
     * Refreshes all status signals.
     * This has the same effect as calling {@link com.ctre.phoenix6.BaseStatusSignal#refreshAll(BaseStatusSignal...)}}}.
     * DO NOT USE if not necessary.
     */
    public void refreshStatusSignals(EncoderSignal... signals) {}

    public boolean configure(EncoderConfiguration encoderConfiguration) { return true; }

    protected void refreshInputs(EncoderInputs inputs) { }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs("Encoders/" + name, inputs);
    }

    @Override
    public EncoderInputs getInputs() {
        return inputs;
    }
}
