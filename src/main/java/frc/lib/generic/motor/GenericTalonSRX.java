package frc.lib.generic.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.Properties;

public class GenericTalonSRX extends WPI_TalonSRX implements Motor {

    public GenericTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        switch (controlMode) {
            case POSITION -> super.set(ControlMode.Position, output);
            case VELOCITY -> super.set(ControlMode.Velocity, output);
            case PERCENTAGE_OUTPUT -> super.set(ControlMode.PercentOutput, output);
            case CURRENT -> super.set(ControlMode.Current, output);
            case VOLTAGE -> super.setVoltage(output);
            case PROFILED_POSITION, PROFILED_VELOCITY -> throw new UnsupportedOperationException("Not implemented yet. go fuck yourself");
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) {
        throw new UnsupportedOperationException("I ain't implementing this lmfao");
    }

    @Override
    public void setP(double kP, int slot) {
        super.config_kP(slot, kP);
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        super.setNeutralMode(idleMode == MotorProperties.IdleMode.COAST ? NeutralMode.Coast : NeutralMode.Brake);
    }

    @Override
    public void setFollowerOf(int masterPort) {
        super.set(ControlMode.Follower, masterPort);
    }

    @Override
    public double getCurrent() {
        return super.getStatorCurrent();
    }

    @Override
    public double getVoltage() {
        return super.getBusVoltage() * 12; //todo: check fi correct
    }

    @Override
    public double getMotorPosition() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public double getMotorVelocity() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public double getSystemPosition() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public double getSystemVelocity() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz) {
        throw new UnsupportedOperationException("Talon SRX does NOT use signals. Use GenericTalonFX instead");
    }

    @Override
    public void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes) {
        throw new UnsupportedOperationException("Talon SRX does NOT use signals. Use GenericTalonFX instead");
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType) {
        throw new UnsupportedOperationException("Talon SRX does NOT use status signals. Use GenericTalonFX instead");
    }

    @Override
    public void refreshStatusSignals(Properties.SignalType... signalTypes) {
        throw new UnsupportedOperationException("Talon SRX does NOT use status signals. Use GenericTalonFX instead");
    }

    @Override
    public TalonFXSimState getSimulationState() {
        throw new UnsupportedOperationException("Simulation is not supported for TalonSRX. Use GenericTalonFX");
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        super.configFactoryDefault();

        configureSlots(configuration);

        TalonSRXConfiguration talonSRXConfiguration = new TalonSRXConfiguration();

        talonSRXConfiguration.openloopRamp = configuration.dutyCycleOpenLoopRampPeriod;
        talonSRXConfiguration.closedloopRamp = configuration.dutyCycleClosedLoopRampPeriod;

        talonSRXConfiguration.feedbackNotContinuous = !configuration.closedLoopContinousWrap;

        talonSRXConfiguration.peakCurrentLimit = (int) configuration.statorCurrentLimit;

        super.setInverted(configuration.inverted);

        return super.configAllSettings(talonSRXConfiguration) == ErrorCode.OK;
    }

    private void configureSlots(MotorConfiguration configuration) {
        super.config_kP(0, configuration.slot0.kP());
        super.config_kI(0, configuration.slot0.kI());
        super.config_kD(0, configuration.slot0.kD());

        super.config_kP(1, configuration.slot1.kP());
        super.config_kI(1, configuration.slot1.kI());
        super.config_kD(1, configuration.slot1.kD());

        super.config_kP(2, configuration.slot2.kP());
        super.config_kI(2, configuration.slot2.kI());
        super.config_kD(2, configuration.slot2.kD());
    }
}
