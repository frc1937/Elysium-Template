package frc.lib.generic.encoder.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderInputs;

import java.util.function.DoubleSupplier;

public class SimulatedCanCoder extends Encoder {
    private DoubleSupplier positionSupplier = () -> 0;
    private DoubleSupplier velocitySupplier = () -> 0;
//todo: IMPLEMENT signalsToLog in here TOO.
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
    protected void refreshInputs(EncoderInputs inputs) {
        if (positionSupplier == null || velocitySupplier == null) {
            return;
        }

        inputs.position = positionSupplier.getAsDouble();
        inputs.velocity = velocitySupplier.getAsDouble();

        inputs.threadPosition = new double[]{inputs.position};
        inputs.threadVelocity = new double[]{inputs.velocity};
        inputs.timestamps = new double[]{Timer.getFPGATimestamp()};
    }
}
