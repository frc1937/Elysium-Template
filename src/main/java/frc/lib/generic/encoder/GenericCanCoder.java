package frc.lib.generic.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.lib.generic.motor.Signal;

import java.util.ArrayList;

/**
 * Wrapper class for the CAN encoder.
 * Verify its setup is correct via this:
 * <a href="https://store.ctr-electronics.com/content/user-manual/CANCoder%20User">CTRE CANcoder PDF</a>'s%20Guide.pdf
 */
public class GenericCanCoder extends CANcoder implements Encoder {
    private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    private final StatusSignal<Double> positionSignal, velocitySignal;

    private final String name;

    public GenericCanCoder(String name, int canCoderID) {
        super(canCoderID);

        this.name = name;

        positionSignal = super.getPosition().clone();
        velocitySignal = super.getVelocity().clone();
    }

    @Override
    public void setSignalUpdateFrequency(Signal signal) {
        final double updateRateHz = signal.getUpdateRate();

        switch (signal.getType()) {
            case POSITION -> positionSignal.setUpdateFrequency(updateRateHz);
            case VELOCITY -> velocitySignal.setUpdateFrequency(updateRateHz);

            case TEMPERATURE, CURRENT, VOLTAGE, CLOSED_LOOP_TARGET ->
                    throw new UnsupportedOperationException("CANCoders don't support checking for " + signal.getName());
        }
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(Signal signal) {
        return switch (signal.getType()) {
            case POSITION -> positionSignal;
            case VELOCITY -> velocitySignal;
            case TEMPERATURE, CURRENT, VOLTAGE, CLOSED_LOOP_TARGET ->
                    throw new UnsupportedOperationException("CANCoders don't support checking for " + signal.getName());
        };
    }

    @Override
    public double getEncoderPosition() {
        return positionSignal.refresh().getValue();
    }

    @Override
    public double getEncoderVelocity() {
        return velocitySignal.refresh().getValue();
    }

    @Override
    public void refreshStatusSignals(Signal... signals) {
        ArrayList<BaseStatusSignal> baseStatusSignals = new ArrayList<>();

        for (Signal signal : signals) {
            baseStatusSignals.add(getRawStatusSignal(signal));
        }

        BaseStatusSignal.refreshAll(baseStatusSignals.toArray(new BaseStatusSignal[0]));
    }

    @Override
    public boolean configure(EncoderConfiguration encoderConfiguration) {
        canCoderConfig.MagnetSensor.MagnetOffset = encoderConfiguration.offsetRotations;

        canCoderConfig.MagnetSensor.SensorDirection = encoderConfiguration.invert ?
                SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfig.MagnetSensor.AbsoluteSensorRange = encoderConfiguration.sensorRange == EncoderProperties.SensorRange.ZeroToOne
                ? AbsoluteSensorRangeValue.Unsigned_0To1 : AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        optimizeBusUtilization();

        return applyConfig();
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = this.getConfigurator().apply(canCoderConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }
}
