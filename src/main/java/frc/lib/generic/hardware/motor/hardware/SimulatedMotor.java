package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.GenericSimulation;
import frc.robot.GlobalConstants;

import java.io.PrintStream;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public class SimulatedMotor extends Motor {
    private MotorConfiguration currentConfiguration;
    private GenericSimulation simulation;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];

    public SimulatedMotor(String name) {
        super(name);

        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION)
            new RuntimeException("DO NOT Initialize THIS MOTOR! Use the factory methods instead!").printStackTrace();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        if (simulation != null) {
            simulation.setOutput(controlMode, output);
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        if (simulation != null) {
            simulation.setOutput(mode, output);
        }
    }

    @Override
    public void stopMotor() {
        if (simulation != null) {
            simulation.stop();
        }
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        simulation = configuration.simulationProperties.getSimulationFromType();

        configuration.slot0 = configuration.simulationSlot;
        configuration.slot1 = configuration.simulationSlot;
        configuration.slot2 = configuration.simulationSlot;

        simulation.configure(configuration);

        return true;
    }

    @Override
    public int getDeviceID() {
        return simulation.getDeviceID();
    }

    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        if (useFasterThread)
            signalsToLog[signal.getId() + MOTOR_INPUTS_LENGTH / 2] = true;

        signalsToLog[signal.getId()] = true;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION) {
            new RuntimeException("This motor should NEVER be initialized manually! Use the factory methods instead!").printStackTrace();
        }

        if (simulation == null) return;

        inputs.setSignalsToLog(signalsToLog);

        inputs.voltage = simulation.getVoltage();
        inputs.current = simulation.getCurrent();
        inputs.temperature = 0;
        inputs.target = simulation.getTarget();
        inputs.systemPosition = simulation.getPositionRotations();
        inputs.systemVelocity = simulation.getVelocityRotationsPerSecond();

        inputs.threadVoltage = new double[]{inputs.voltage};
        inputs.threadCurrent = new double[]{inputs.current};
        inputs.threadTemperature = new double[]{inputs.temperature};
        inputs.threadTarget = new double[]{inputs.target};
        inputs.threadSystemPosition = new double[]{inputs.systemPosition};
        inputs.threadSystemVelocity = new double[]{inputs.systemVelocity};
    }
}
