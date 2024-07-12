package frc.lib.generic.motor.hardware;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.MotorSignal;

import java.util.function.DoubleSupplier;

public class GenericTalonSRX implements Motor {
    private final WPI_TalonSRX talonSRX;

    private MotorConfiguration currentConfiguration;
    private int slotToUse = 0;

    private String name;

    public GenericTalonSRX(String name, int deviceNumber) {
        talonSRX = new WPI_TalonSRX(deviceNumber);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        switch (controlMode) {
            case POSITION -> talonSRX.set(ControlMode.Position, output);
            case VELOCITY -> talonSRX.set(ControlMode.Velocity, output);
            case PERCENTAGE_OUTPUT -> talonSRX.set(ControlMode.PercentOutput, output);
            case CURRENT -> talonSRX.set(ControlMode.Current, output);
            case VOLTAGE -> talonSRX.setVoltage(output);
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) {
        throw new UnsupportedOperationException("I ain't implementing this lmfao");
    }

    @Override
    public void setExternalPositionSupplier(DoubleSupplier position) {
        //todo: do
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocity) {
        //todo: implement
    }

    @Override
    public MotorProperties.Slot getCurrentSlot() {
        return getSlot(slotToUse, currentConfiguration);
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void resetSlot(MotorProperties.Slot slot, int slotNumber) {
        talonSRX.config_kP(slotNumber, slot.kP());
        talonSRX.config_kI(slotNumber, slot.kI());
        talonSRX.config_kD(slotNumber, slot.kD());
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        talonSRX.setNeutralMode(idleMode == MotorProperties.IdleMode.COAST ? NeutralMode.Coast : NeutralMode.Brake);
    }

    @Override
    public void stopMotor() {
        talonSRX.stopMotor();
    }

    @Override
    public void setFollowerOf(String name, int masterPort) {
        talonSRX.set(ControlMode.Follower, masterPort);
    }

    @Override
    public double getCurrent() {
        return talonSRX.getStatorCurrent();
    }

    @Override
    public double getVoltage() {
        return talonSRX.getBusVoltage() * 12; //todo: check fi correct
    }

    @Override
    public double getClosedLoopTarget() {
        return talonSRX.getClosedLoopTarget();
    }

    @Override
    public double getTemperature() {
        return talonSRX.getTemperature();
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
    public int getDeviceID() {
        return talonSRX.getDeviceID();
    }

    @Override
    public void setupSignalUpdates(MotorSignal signal) {
        throw new UnsupportedOperationException("Talon SRX does NOT use signals. Use GenericTalonFX instead");
    }

    @Override
    public void setupSignalsUpdates(MotorSignal... signal) {
        throw new UnsupportedOperationException("Talon SRX does NOT use signals. Use GenericTalonFX instead");
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(MotorSignal signal) {
        throw new UnsupportedOperationException("Talon SRX does NOT use status signals. Use GenericTalonFX instead");
    }

    @Override
    public void refreshStatusSignals(MotorSignal... signal) {
        throw new UnsupportedOperationException("Talon SRX does NOT use status signals. Use GenericTalonFX instead");
    }

    @Override
    public TalonFXSimState getSimulationState() {
        throw new UnsupportedOperationException("Simulation is not supported for TalonSRX. Use GenericTalonFX");
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        talonSRX.configFactoryDefault();

        currentConfiguration = configuration;
        slotToUse = configuration.slotToUse;

        configureSlots(configuration);

        TalonSRXConfiguration talonSRXConfiguration = new TalonSRXConfiguration();

        talonSRXConfiguration.openloopRamp = configuration.dutyCycleOpenLoopRampPeriod;
        talonSRXConfiguration.closedloopRamp = configuration.dutyCycleClosedLoopRampPeriod;

        talonSRXConfiguration.feedbackNotContinuous = !configuration.closedLoopContinuousWrap;

        talonSRXConfiguration.peakCurrentLimit = (int) configuration.statorCurrentLimit;

        talonSRX.setInverted(configuration.inverted);

        return talonSRX.configAllSettings(talonSRXConfiguration) == ErrorCode.OK;
    }


    private void configureSlots(MotorConfiguration configuration) {
        talonSRX.config_kP(0, configuration.slot0.kP());
        talonSRX.config_kI(0, configuration.slot0.kI());
        talonSRX.config_kD(0, configuration.slot0.kD());

        talonSRX.config_kP(1, configuration.slot1.kP());
        talonSRX.config_kI(1, configuration.slot1.kI());
        talonSRX.config_kD(1, configuration.slot1.kD());

        talonSRX.config_kP(2, configuration.slot2.kP());
        talonSRX.config_kI(2, configuration.slot2.kI());
        talonSRX.config_kD(2, configuration.slot2.kD());
    }
}
