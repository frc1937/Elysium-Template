package frc.lib.generic.hardware.encoder.hardware;

import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderInputs;
import frc.lib.generic.hardware.encoder.EncoderSignal;

import java.util.function.DoubleSupplier;

public class SimulatedCanCoder extends Encoder {
    private DoubleSupplier positionSupplier = () -> 0;
    private DoubleSupplier velocitySupplier = () -> 0;

    private final boolean[] signalsToLog = new boolean[4];

    public SimulatedCanCoder(String name) {
        super(name);
    }

    @Override
    public void setSignalUpdateFrequency(EncoderSignal signal) {
        signalsToLog[signal.getType().getId()] = true;

        if (signal.useFasterThread()) {
            signalsToLog[signal.getType().getId() + 2] = true;
        }
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
    protected void refreshInputs(EncoderInputs inputs) {
        inputs.setSignalsToLog(signalsToLog);

        if (positionSupplier == null || velocitySupplier == null) {
            return;
        }

        inputs.position = positionSupplier.getAsDouble();
        inputs.velocity = velocitySupplier.getAsDouble();

        inputs.threadPosition = new double[]{inputs.position};
        inputs.threadVelocity = new double[]{inputs.velocity};
    }
}
