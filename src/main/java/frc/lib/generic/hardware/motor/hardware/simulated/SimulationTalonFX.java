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
import frc.lib.generic.hardware.motor.MotorSignal;

import static frc.lib.generic.hardware.motor.MotorProperties.GravityType.ARM;

public class SimulationTalonFX extends Motor {
    private final TalonFX talonFX;

    private final StatusSignal<Double> positionSignal, velocitySignal, accelerationSignal, voltageSignal, currentSignal, temperatureSignal;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private MotorConfiguration currentConfiguration;

    private boolean shouldUseProfile = false;
    private int slotToUse = 0;

    private double target = 0;

    public SimulationTalonFX(String name, int deviceId) {
        super(name);

        talonFX = new TalonFX(deviceId);

        talonConfigurator = talonFX.getConfigurator();

        positionSignal = talonFX.getPosition().clone();
        velocitySignal = talonFX.getVelocity().clone();
        accelerationSignal = talonFX.getAcceleration().clone();
        voltageSignal = talonFX.getMotorVoltage().clone();
        currentSignal = talonFX.getStatorCurrent().clone();
        temperatureSignal = talonFX.getDeviceTemp().clone();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        target = output;

        switch (mode) {
            case VOLTAGE -> talonFX.setControl(voltageRequest.withOutput(output));

            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(slotToUse));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(slotToUse));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(slotToUse));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(slotToUse));
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

    public TalonFXSimState getSimulationState() {
        return talonFX.getSimState();
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        this.currentConfiguration = configuration;

        configureMotionMagic();

        setConfig0();
        setConfig1();
        setConfig2();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

        slotToUse = configuration.slotToUse;

        return applyConfig();
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

    private void setConfig1() {
        talonConfig.Slot1.kP = currentConfiguration.slot1.kP();
        talonConfig.Slot1.kI = currentConfiguration.slot1.kI();
        talonConfig.Slot1.kD = currentConfiguration.slot1.kD();
        talonConfig.Slot1.kA = currentConfiguration.slot1.kA();
        talonConfig.Slot1.kS = currentConfiguration.slot1.kS();
        talonConfig.Slot1.kV = currentConfiguration.slot1.kV();
        talonConfig.Slot1.kG = currentConfiguration.slot1.kG();

        if (currentConfiguration.slot1.gravityType() != null)
            talonConfig.Slot1.GravityType = currentConfiguration.slot1.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
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
            talonConfig.Slot2.GravityType = currentConfiguration.slot2.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
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

    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        switch (signal) {
            case ACCELERATION -> accelerationSignal.setUpdateFrequency(1000);
            case VELOCITY -> velocitySignal.setUpdateFrequency(1000);
            case POSITION -> positionSignal.setUpdateFrequency(1000);
            case VOLTAGE -> voltageSignal.setUpdateFrequency(1000);
            case CURRENT -> currentSignal.setUpdateFrequency(1000);
            case TEMPERATURE -> temperatureSignal.setUpdateFrequency(1000);
        }
    }
}
