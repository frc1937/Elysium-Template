package frc.lib.generic.hardware.encoder.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderInputs;
import frc.lib.generic.hardware.encoder.EncoderSignal;

import java.util.function.DoubleSupplier;

public class SimulatedCanCoder extends Encoder {
    private final boolean[] signalsToLog = new boolean[EncoderInputs.ENCODER_INPUTS_LENGTH];

    private DoubleSupplier positionSupplier = () -> 0;
    private DoubleSupplier velocitySupplier = () -> 0;

    public SimulatedCanCoder(String name) {
        super(name);
    }

    @Override
    public void setSimulatedEncoderPositionSource(DoubleSupplier supplier) {
        positionSupplier = supplier;
    }

    @Override
    public void setSimulatedEncoderVelocitySource(DoubleSupplier supplier) {
        velocitySupplier = supplier;
    }

    @Override
    public void setSignalUpdateFrequency(EncoderSignal signal) {
        signalsToLog[signal.getType().getId()] = true;

        if (signal.useFasterThread()) {
            signalsToLog[2] = true;
            signalsToLog[signal.getType().getId() + 3] = true;
        }
    }

    @Override
    protected void refreshInputs(EncoderInputs inputs) {
        if (positionSupplier == null || velocitySupplier == null) {
            return;
        }

        inputs.setSignalsToLog(signalsToLog);

        inputs.position = positionSupplier.getAsDouble();
        inputs.velocity = velocitySupplier.getAsDouble();

        inputs.threadPosition = new double[]{inputs.position};
        inputs.threadVelocity = new double[]{inputs.velocity};
        inputs.timestamps = new double[]{Timer.getFPGATimestamp()};
    }
}
