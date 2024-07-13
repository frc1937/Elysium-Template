package frc.lib.generic.simulation;

import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.MotorSignal;
import frc.lib.generic.motor.hardware.GenericTalonFX;
import frc.robot.GlobalConstants;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public abstract class GenericSimulation {
    /**
     * This instance is shared between all inheritors
     */
    private static final List<GenericSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    private Motor motor;
    private TalonFXSimState motorSimulationState;

    protected GenericSimulation() {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL) return;

        REGISTERED_SIMULATIONS.add(this);

        motor = new GenericTalonFX("Amit Sucher", REGISTERED_SIMULATIONS.size() - 1);

        //This is simulation. we don't give a damn fuck! about performance.
        MotorSignal[] signals = new MotorSignal[]{
                new MotorSignal(MotorSignal.SignalType.POSITION, true), new MotorSignal(MotorSignal.SignalType.VELOCITY, true),
                new MotorSignal(MotorSignal.SignalType.CURRENT, true), new MotorSignal(MotorSignal.SignalType.VOLTAGE, true),
                new MotorSignal(MotorSignal.SignalType.TEMPERATURE, true), new MotorSignal(MotorSignal.SignalType.CLOSED_LOOP_TARGET, true)
        };

        motor.setupSignalsUpdates(signals);

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

    public double getTarget() {
        return motor.getClosedLoopTarget();
    }

    public abstract double getPositionRotations();

    public abstract double getVelocityRotationsPerSecond();

    public abstract double getCurrent();

    //These have weaker access because they're used in this package only.
    abstract void update();

    abstract void setVoltage(double voltage);
}
