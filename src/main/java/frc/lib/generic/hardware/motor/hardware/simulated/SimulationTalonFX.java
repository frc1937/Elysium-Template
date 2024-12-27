package frc.lib.generic.hardware.motor.hardware.simulated;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.GravityType.ARM;

public class SimulationTalonFX extends Motor {
    private final TalonFX talonFX;

    private final StatusSignal<Double> voltageSignal;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private MotorConfiguration currentConfiguration;

    private boolean shouldUseProfile = false;
    private double target = 0;

    public SimulationTalonFX(String name, int deviceId) {
        super(name);

        talonFX = new TalonFX(deviceId);

        talonConfigurator = talonFX.getConfigurator();

        voltageSignal = talonFX.getMotorVoltage().clone();
        voltageSignal.setUpdateFrequency(1000);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        target = output;

        switch (mode) {
            case VOLTAGE -> talonFX.setControl(voltageRequest.withOutput(output));

            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(0));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(0));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(0));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(0));
                }
            }

            case CURRENT ->
                    new UnsupportedOperationException("CTRE LOVES money and wants $150!!! dollars for this.. wtf.").printStackTrace();
        }
    }

    @Override
    public double getVoltage() {
        return voltageSignal.refresh().getValue();
    }

    @Override
    public double getClosedLoopTarget() {
        return target;
    }

    @Override
    public void stopMotor() {
        talonFX.stopMotor();
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        this.currentConfiguration = configuration;

        configureMotionMagic();

        setConfig0();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

        return applyConfig();
    }

    public TalonFXSimState getSimulationState() {
        return talonFX.getSimState();
    }

    private void configureMotionMagic() {
        if (currentConfiguration.profileMaxVelocity == 0 && currentConfiguration.profileMaxAcceleration == 0 && currentConfiguration.profileMaxJerk == 0 ||
                currentConfiguration.profileMaxVelocity != 0 && currentConfiguration.profileMaxAcceleration == 0 && currentConfiguration.profileMaxJerk == 0)
            return;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = currentConfiguration.profileMaxVelocity;
        talonConfig.MotionMagic.MotionMagicAcceleration = currentConfiguration.profileMaxAcceleration;
        talonConfig.MotionMagic.MotionMagicJerk = currentConfiguration.profileMaxJerk;

        shouldUseProfile = true;
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
            talonConfig.Slot0.GravityType = currentConfiguration.slot0.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
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
