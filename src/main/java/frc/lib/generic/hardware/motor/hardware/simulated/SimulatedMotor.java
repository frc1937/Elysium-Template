package frc.lib.generic.hardware.motor.hardware.simulated;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.GenericPhysicsSimulation;
import frc.robot.GlobalConstants;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;
import static frc.lib.generic.hardware.motor.MotorProperties.GravityType.ARM;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public class SimulatedMotor extends Motor {
    private MotorConfiguration currentConfiguration;
    private GenericPhysicsSimulation simulation;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];

    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSimState;

    private final StatusSignal<Double> voltageSignal;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private boolean shouldUseProfile = false;
    private double target = 0;

    public SimulatedMotor(String name, int port) {
        super(name);

        talonFX = new TalonFX(port);

        talonFXSimState = talonFX.getSimState();
        talonFXSimState.setSupplyVoltage(12);

        talonConfigurator = talonFX.getConfigurator();

        voltageSignal = talonFX.getMotorVoltage().clone();
        voltageSignal.setUpdateFrequency(1000);

        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION)
            new RuntimeException("DO NOT Initialize THIS MOTOR! Use the factory methods instead!").printStackTrace();
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
    public void stopMotor() {
        talonFX.stopMotor();
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        simulation = configuration.simulationProperties.getSimulationType();

        if (simulation == null) {
            new RuntimeException("Simulation configurations not found for motor: " + getName()).printStackTrace();
            return false;
        }

        configureMotionMagic();
        setTalonConfig();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

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

    private void setTalonConfig() {
        talonConfig.Slot0.kP = currentConfiguration.simulationSlot.kP();
        talonConfig.Slot0.kI = currentConfiguration.simulationSlot.kI();
        talonConfig.Slot0.kD = currentConfiguration.simulationSlot.kD();

        talonConfig.Slot0.kA = currentConfiguration.simulationSlot.kA();
        talonConfig.Slot0.kS = currentConfiguration.simulationSlot.kS();
        talonConfig.Slot0.kV = currentConfiguration.simulationSlot.kV();
        talonConfig.Slot0.kG = currentConfiguration.simulationSlot.kG();

        if (currentConfiguration.simulationSlot.gravityType() != null)
            talonConfig.Slot0.GravityType = currentConfiguration.simulationSlot.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
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
    public int getDeviceID() {
        return talonFX.getDeviceID();
    }

    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        if (useFasterThread)
            signalsToLog[signal.getId() + MOTOR_INPUTS_LENGTH / 2] = true;

        signalsToLog[signal.getId()] = true;
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION) {
            new RuntimeException("This motor should NEVER be initialized manually! Use the factory methods instead!").printStackTrace();
        }

        if (simulation == null) return;

        inputs.setSignalsToLog(signalsToLog);

        inputs.voltage = voltageSignal.refresh().getValue();
        inputs.current = simulation.getCurrent();
        inputs.temperature = 0;
        inputs.target = target;
        inputs.systemPosition = simulation.getSystemPositionRotations();
        inputs.systemVelocity = simulation.getSystemVelocityRotationsPerSecond();
        inputs.systemAcceleration = simulation.getSystemAccelerationRotationsPerSecondSquared();

        inputs.threadVoltage = new double[]{inputs.voltage};
        inputs.threadCurrent = new double[]{inputs.current};
        inputs.threadTemperature = new double[]{inputs.temperature};
        inputs.threadTarget = new double[]{inputs.target};
        inputs.threadSystemPosition = new double[]{inputs.systemPosition};
        inputs.threadSystemVelocity = new double[]{inputs.systemVelocity};
        inputs.threadSystemAcceleration = new double[]{inputs.systemAcceleration};
    }

    public void updateSimulation() {
        simulation.setVoltage(talonFXSimState.getMotorVoltage());
        simulation.updateMotor();

        talonFXSimState.setRawRotorPosition(simulation.getSystemPositionRotations());
        talonFXSimState.setRotorVelocity(simulation.getSystemVelocityRotationsPerSecond());
        talonFXSimState.setRotorAcceleration(simulation.getSystemAccelerationRotationsPerSecondSquared());
    }
}