package frc.lib.generic.simulation;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.generic.Properties;
import frc.lib.generic.motor.GenericTalonFX;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.util.TunableNumber;

import java.util.ArrayList;
import java.util.List;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public abstract class GenericSimulation {
    /** This instance is shared between all inheritors */
    private static final List<GenericSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    private final TunableNumber volts = new TunableNumber("ngig", 0);

    private final Motor motor;
    private TalonFXSimState motorSimulationState;

    GenericSimulation() {
        REGISTERED_SIMULATIONS.add(this);

        motor = new GenericTalonFX(REGISTERED_SIMULATIONS.size() - 1);

        motor.setSignalUpdateFrequency(Properties.SignalType.CLOSED_LOOP_TARGET, 1.0 / ROBORIO_LOOP_TIME);
        motor.setSignalUpdateFrequency(Properties.SignalType.VOLTAGE, 1.0 / ROBORIO_LOOP_TIME);

        motor.setSignalUpdateFrequency(Properties.SignalType.POSITION, 1.0 / ROBORIO_LOOP_TIME);
        motor.setSignalUpdateFrequency(Properties.SignalType.VELOCITY, 1.0 / ROBORIO_LOOP_TIME);
        motor.setSignalUpdateFrequency(Properties.SignalType.CURRENT, 1.0 / ROBORIO_LOOP_TIME);

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
        SmartDashboard.putNumber("POSITION HIGHER LEVEL! 2: ", output);
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    private void updateSimulation() {
        motorSimulationState = motor.getSimulationState();

        setVoltage(volts.get());
        SmartDashboard.putNumber("voltage of mtoor", motorSimulationState.getMotorVoltage());
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
