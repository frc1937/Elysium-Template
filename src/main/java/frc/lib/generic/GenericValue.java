package frc.lib.generic;

import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

/**
 * Represents a value that can differ between simulation and real execution modes.
 * It holds separate values for each mode and returns the appropriate one based on the current mode.
 * @param <T> The type of the values this class holds.
 */
public final class GenericValue<T> {
    private final T realValue;
    private final T simulationValue;

    public GenericValue(T value) {
        this(value, value);
    }

    /**
     * Constructs a new GenericValue with specified real and simulation values.
     * @param realValue The value to be used in real execution mode.
     * @param simulationValue The value to be used in simulation mode.
     */
    public GenericValue(T realValue, T simulationValue) {
        this.realValue = realValue;
        this.simulationValue = simulationValue;
    }

    /**
     * Returns the value appropriate for the current execution mode.
     * @return The real value if in real execution mode, or the simulation value if in simulation mode.
     */
    public T get() {
        return CURRENT_MODE == GlobalConstants.Mode.SIMULATION ? simulationValue : realValue;
    }
}
