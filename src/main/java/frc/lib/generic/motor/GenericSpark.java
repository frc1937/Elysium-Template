package frc.lib.generic.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import org.littletonrobotics.junction.Logger;

public class GenericSpark extends CANSparkBase implements Motor {
    private final MotorProperties.SparkType model;

    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    private double closedLoopTarget;
    private Feedforward feedforward;

    private int slotToUse = 0;

    public GenericSpark(int deviceId, MotorProperties.SparkType sparkType) {
        super(deviceId, MotorType.kBrushless, sparkType == MotorProperties.SparkType.MAX ? SparkModel.SparkMax : SparkModel.SparkFlex);
        model = sparkType;

        optimizeBusUsage();

        encoder = getEncoder();
        controller = super.getPIDController();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output,
                this.feedforward.calculate(getSystemPosition(), getSystemVelocity(), 0));
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;

        switch (mode) {
            case PERCENTAGE_OUTPUT -> controller.setReference(output, ControlType.kDutyCycle);

            case VELOCITY -> controller.setReference(output * 60, ControlType.kVelocity, slotToUse, feedforward);
            case POSITION -> controller.setReference(output, ControlType.kPosition, slotToUse, feedforward);

            case VOLTAGE -> controller.setReference(output, ControlType.kVoltage, slotToUse);
            case CURRENT -> controller.setReference(output, ControlType.kCurrent, slotToUse);

            case PROFILED_POSITION, PROFILED_VELOCITY ->
                    throw new UnsupportedOperationException("Use normal VOLTAGE control and create the profile outside the motor.");
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? IdleMode.kCoast : IdleMode.kBrake);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        encoder.setPosition(position);
        Logger.recordOutput("EncoderValue" + getDeviceID(), encoder.getPosition());
        //Position to set the motor to, after gear ratio is applied
    }

    @Override
    public int getDeviceID() {
        return getDeviceId();
    }

    @Override
    public void setP(double kP, int slot) {
        controller.setP(kP, slot);
    }

    @Override
    public double getMotorPosition() {
        return getSystemPosition() / encoder.getPositionConversionFactor();
    }

    @Override
    public double getMotorVelocity() {
        return encoder.getVelocity() / 60 * encoder.getVelocityConversionFactor();
    }

    @Override
    public double getCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public double getTemperature() {
        return getMotorTemperature();
    }

    @Override
    public double getSystemPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getSystemVelocity() {
        return encoder.getVelocity() / 60;
    }

    @Override
    public double getClosedLoopTarget() {
        return closedLoopTarget;
    }

    @Override
    public double getVoltage() {
        return super.getBusVoltage();
    }


    @Override
    public void setFollowerOf(int masterPort) {
        super.follow(new GenericSpark(masterPort, model));
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">REV DOCS</a>
     */
    @Override
    public void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz) {
        int ms = (int) (1000 / updateFrequencyHz);

        switch (signalType) {
            case VELOCITY, CURRENT -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
            case POSITION -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, ms);
        }
    }

    @Override
    public void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes) {
        for (Properties.SignalType signalType : signalTypes) {
            setSignalUpdateFrequency(signalType, updateFrequencyHz);
        }
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public void refreshStatusSignals(Properties.SignalType... signalTypes) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public TalonFXSimState getSimulationState() {
        throw new UnsupportedOperationException("Not supported. Use GenericTalonFX");
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        super.restoreFactoryDefaults();

        super.setIdleMode(configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? IdleMode.kBrake : IdleMode.kCoast);
        super.setInverted(configuration.inverted);

        encoder.setPositionConversionFactor(1 / configuration.gearRatio);
        encoder.setVelocityConversionFactor(1 / configuration.gearRatio);

        super.enableVoltageCompensation(12);

        if (configuration.statorCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configurePID(configuration);
        configureFeedForward(configuration);

        return super.burnFlash() == REVLibError.kOk;
    }

    private void configureFeedForward(MotorConfiguration configuration) {
        MotorProperties.Slot currentSlot = configuration.slot0;

        switch (slotToUse) {
            case 1 -> currentSlot = configuration.slot1;
            case 2 -> currentSlot = configuration.slot2;
        }

        if (currentSlot.gravityType() == null) {
            feedforward = new Feedforward(Properties.FeedforwardType.SIMPLE,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA()
            );
        }

        if (currentSlot.gravityType() == GravityTypeValue.Arm_Cosine) {
            feedforward = new Feedforward(Properties.FeedforwardType.ARM,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA(),
                    currentSlot.kG()
            );
        }

        if (currentSlot.gravityType() == GravityTypeValue.Elevator_Static) {
            feedforward = new Feedforward(Properties.FeedforwardType.ELEVATOR,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA(),
                    currentSlot.kG()
            );
        }
    }

    private void configurePID(MotorConfiguration configuration) {
        controller.setPositionPIDWrappingEnabled(configuration.closedLoopContinousWrap);

        controller.setP(configuration.slot0.kP(), 0);
        controller.setI(configuration.slot0.kI(), 0);
        controller.setD(configuration.slot0.kD(), 0);

        controller.setP(configuration.slot1.kP(), 1);
        controller.setI(configuration.slot1.kI(), 1);
        controller.setD(configuration.slot1.kD(), 1);

        controller.setP(configuration.slot2.kP(), 2);
        controller.setI(configuration.slot2.kI(), 2);
        controller.setD(configuration.slot2.kD(), 2);

        slotToUse = configuration.slotToUse;
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
    }

    @Override
    public RelativeEncoder getEncoder() {
        switch (model) {
            case MAX -> {
                return getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
            }

            case FLEX -> {
                return getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
            }
        }

        return null;
    }
}
