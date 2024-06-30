package frc.lib.generic.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.Properties;

import java.util.ArrayList;

public class GenericTalonFX extends TalonFX implements Motor {
    private int slotToUse = 0;

    private final StatusSignal<Double> positionSignal, velocitySignal, voltageSignal, currentSignal, temperatureSignal, closedLoopTarget;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private MotorConfiguration currentConfiguration;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    public GenericTalonFX(int deviceId) {
        super(deviceId);

        talonConfigurator = super.getConfigurator();

        positionSignal = super.getPosition().clone();
        velocitySignal = super.getVelocity().clone();
        voltageSignal = super.getMotorVoltage().clone();
        currentSignal = super.getStatorCurrent().clone();
        temperatureSignal = super.getDeviceTemp().clone();
        closedLoopTarget = super.getClosedLoopReference().clone();
    }

    @Override
    public void setInput(MotorProperties.ControlMode mode, double input) {
        switch (mode) { //todo: add more (NECESSARY) control types
            case PERCENTAGE_OUTPUT -> this.setControl(dutyCycleRequest.withOutput(input));
            case VOLTAGE -> this.setControl(voltageRequest.withOutput(input).withEnableFOC(false));

            case POSITION -> this.setControl(positionVoltageRequest.withPosition(input).withSlot(slotToUse));
            case VELOCITY -> this.setControl(velocityVoltageRequest.withVelocity(input).withSlot(slotToUse));

            case CURRENT ->
                    throw new UnsupportedOperationException("CTRE is money hungry, and wants you to pay $150 for CURRENT control. Nuh uh!");
        }
    }

    @Override
    public void setInput(MotorProperties.ControlMode mode, double input, double feedforward) {
        if (mode != MotorProperties.ControlMode.POSITION && mode != MotorProperties.ControlMode.VELOCITY)
            setInput(mode, input);

        switch (mode) {
            case POSITION -> super.setControl(positionVoltageRequest.withPosition(input).withSlot(slotToUse)
                    .withFeedForward(feedforward));
            case VELOCITY -> super.setControl(velocityVoltageRequest.withVelocity(input).withSlot(slotToUse)
                    .withFeedForward(feedforward));
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        currentConfiguration.idleMode = idleMode;
        configure(currentConfiguration);
    }

    @Override
    public void setP(double kP, int slot) {
        if (slot == 0 && talonConfig.Slot0.kP == kP) return;
        if (slot == 1 && talonConfig.Slot1.kP == kP) return;
        if (slot == 2 && talonConfig.Slot2.kP == kP) return;

        switch (slot) {
            case 0 -> talonConfig.Slot0.kP = kP;
            case 1 -> talonConfig.Slot1.kP = kP;
            case 2 -> talonConfig.Slot2.kP = kP;
        }

        applyConfig();
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        talonConfigurator.setPosition(position); //TODO: Test on real robot to check if works.
    }

    @Override
    public double getMotorVelocity() {
        return getSystemVelocity() / talonConfig.Feedback.SensorToMechanismRatio;
    }

    @Override
    public double getMotorPosition() {
        return getSystemPosition() / talonConfig.Feedback.SensorToMechanismRatio;
    }

    @Override
    public double getSystemPosition() {
        return positionSignal.refresh().getValue();
    }

    @Override
    public double getClosedLoopTarget() {
        return closedLoopTarget.refresh().getValue();
    }

    @Override
    public double getSystemVelocity() {
        return velocitySignal.refresh().getValue();
    }

    @Override
    public double getTemperature() {
        return temperatureSignal.refresh().getValue();
    }

    @Override
    public double getCurrent() {
        return currentSignal.refresh().getValue();
    }

    @Override
    public double getVoltage() {
        return getMotorVoltage().getValue();
    }

    @Override
    public void setFollowerOf(int masterPort) {
        setControl(new StrictFollower(masterPort)); //check if this should be called 1 times or once is enough
    }

    @Override
    public void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz) {
        switch (signalType) {
            case VELOCITY -> velocitySignal.setUpdateFrequency(updateFrequencyHz);
            case POSITION -> positionSignal.setUpdateFrequency(updateFrequencyHz);
            case VOLTAGE -> voltageSignal.setUpdateFrequency(updateFrequencyHz);
            case CURRENT -> currentSignal.setUpdateFrequency(updateFrequencyHz);
            case TEMPERATURE -> temperatureSignal.setUpdateFrequency(updateFrequencyHz);
            case CLOSED_LOOP_TARGET -> closedLoopTarget.setUpdateFrequency(updateFrequencyHz);
        }
    }

    @Override
    public void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes) {
        for (Properties.SignalType type : signalTypes) {
            setSignalUpdateFrequency(type, updateFrequencyHz);
        }
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType) {
        return switch (signalType) {
            case VELOCITY -> velocitySignal;
            case POSITION -> positionSignal;
            case VOLTAGE -> voltageSignal;
            case CURRENT -> currentSignal;
            case TEMPERATURE -> temperatureSignal;
            case CLOSED_LOOP_TARGET -> closedLoopTarget;
        };
    }

    @Override
    public void refreshStatusSignals(Properties.SignalType... signalTypes) {
        ArrayList<BaseStatusSignal> signals = new ArrayList<>();

        for (Properties.SignalType signalType : signalTypes) {
            signals.add(getRawStatusSignal(signalType));
        }

        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));
    }

    @Override
    public TalonFXSimState getSimulationState() {
        return this.getSimState();
    }


    @Override
    public boolean configure(MotorConfiguration configuration) {
        this.currentConfiguration = configuration;

        talonConfig.MotorOutput.Inverted = configuration.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //Who the FUCK added this feature. CTRE should fucking fire him bruh
        talonConfig.Audio.BeepOnBoot = false;
        talonConfig.Audio.BeepOnConfig = false;

        talonConfig.Voltage.PeakForwardVoltage = 12;
        talonConfig.Voltage.PeakReverseVoltage = -12;

        talonConfig.Feedback.SensorToMechanismRatio = configuration.conversionFactor;

        setConfig0();
        setConfig1();
        setConfig2();

        applyCurrentLimits();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.ClosedLoopContinousWrap;

        slotToUse = configuration.slotToUse;

        optimizeBusUtilization();

        return applyConfig();
    }

    private void setConfig0() {
        talonConfig.Slot0.kP = currentConfiguration.slot0.kP();
        talonConfig.Slot0.kI = currentConfiguration.slot0.kI();
        talonConfig.Slot0.kD = currentConfiguration.slot0.kD();

        talonConfig.Slot0.kA = currentConfiguration.slot0.kA();
        talonConfig.Slot0.kS = currentConfiguration.slot0.kS();
        talonConfig.Slot0.kV = currentConfiguration.slot0.kV();
        talonConfig.Slot0.kG = currentConfiguration.slot0.kG();

        if (currentConfiguration.slot0.gravityType() != null)
            talonConfig.Slot0.GravityType = currentConfiguration.slot0.gravityType();
    }

    private void setConfig1() {
        talonConfig.Slot1.kP = currentConfiguration.slot1.kP();
        talonConfig.Slot1.kI = currentConfiguration.slot1.kI();
        talonConfig.Slot1.kD = currentConfiguration.slot1.kD();
        talonConfig.Slot1.kA = currentConfiguration.slot1.kA();
        talonConfig.Slot1.kS = currentConfiguration.slot1.kS();
        talonConfig.Slot1.kV = currentConfiguration.slot1.kV();
        talonConfig.Slot1.kG = currentConfiguration.slot1.kG();

        if (currentConfiguration.slot1.gravityType() != null)
            talonConfig.Slot1.GravityType = currentConfiguration.slot1.gravityType();
    }

    private void setConfig2() {
        talonConfig.Slot2.kP = currentConfiguration.slot2.kP();
        talonConfig.Slot2.kI = currentConfiguration.slot2.kI();
        talonConfig.Slot2.kD = currentConfiguration.slot2.kD();
        talonConfig.Slot2.kA = currentConfiguration.slot2.kA();
        talonConfig.Slot2.kS = currentConfiguration.slot2.kS();
        talonConfig.Slot2.kV = currentConfiguration.slot2.kV();
        talonConfig.Slot2.kG = currentConfiguration.slot2.kG();

        if (currentConfiguration.slot2.gravityType() != null)
            talonConfig.Slot2.GravityType = currentConfiguration.slot2.gravityType();
    }

    private void applyCurrentLimits() {
        talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = currentConfiguration.dutyCycleOpenLoopRampPeriod;
        talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = currentConfiguration.dutyCycleCloseLoopRampPeriod;

        if (currentConfiguration.statorCurrentLimit != -1) {
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfig.CurrentLimits.StatorCurrentLimit = currentConfiguration.statorCurrentLimit;
        }

        if (currentConfiguration.supplyCurrentLimit != -1) {
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            talonConfig.CurrentLimits.SupplyCurrentLimit = currentConfiguration.supplyCurrentLimit;
        }
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = talonConfigurator.apply(talonConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }
}
