package frc.lib.generic.simulation;

import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.Properties;
import frc.lib.generic.motor.GenericTalonFX;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

import java.util.ArrayList;
import java.util.List;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public abstract class GenericSimulation {
    /** This instance is shared between all inheritors */
    private static final List<GenericSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    private final Motor motor;
    private final TalonFXSimState motorSimulationState;

    protected GenericSimulation() {
        REGISTERED_SIMULATIONS.add(this);

        motor = new GenericTalonFX(REGISTERED_SIMULATIONS.size() - 1);

        //This is simulation. we don't give a damn fuck! about performance.
        Properties.SignalType[] signalTypes = Properties.SignalType.values();

        motor.setSignalsUpdateFrequency(1.0 / ROBORIO_LOOP_TIME, signalTypes);

        motorSimulationState = motor.getSimulationState();
        motorSimulationState.setSupplyVoltage(12); //Voltage compensation.
    }

    /**
     * This method should be called on the @simulationPeriodic
     */
    public static void updateAllSimulations() {
        for (GenericSimulation simulation : REGISTERED_SIMULATIONS) {
            simulation.updateSimulation();
        }
    }

    public void configure(MotorConfiguration configuration) {
        motor.configure(configuration);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        motor.setOutput(controlMode, output);
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    private void updateSimulation() {
        setVoltage(motorSimulationState.getMotorVoltage());
        update();

        motorSimulationState.setRawRotorPosition(getPositionRotations());
        motorSimulationState.setRotorVelocity(getVelocityRotationsPerSecond());
    }

    public abstract double getPositionRotations();
    public abstract double getVelocityRotationsPerSecond();
    public abstract double getCurrent();

    //These have weaker access because they're used here only.
    abstract void update();
    abstract void setVoltage(double voltage);
}
