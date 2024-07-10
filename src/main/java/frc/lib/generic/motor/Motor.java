package frc.lib.generic.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.Properties;

/**
 * Custom Motor class to allow switching and replacing motors quickly,
 * in addition of better uniformity across the code.
 */
public interface Motor {

    /**
     * Gets the currently used configuration slot used by the motor. If this is not set, it will return null.
     *
     * @return The configuration slot
     */
    MotorProperties.Slot getCurrentSlot();

    /**
     * Gets the currently used configuration used by the motor. If this is not set, it will return null.
     *
     * @return The configuration
     */
    MotorConfiguration getCurrentConfiguration();

    /**
     * In case you need to re-set the slot on runtime, use this.
     *
     * @param slot       The new slot values
     * @param slotNumber The slot number to modify
     */
    void resetSlot(MotorProperties.Slot slot, int slotNumber);

    /**
     * Sets the output of the motor based on the specified control mode and desired output value.
     *
     * <p>This method utilizes the built-in feedforward and PID controller to achieve precise control
     * over the motor. The control mode determines how the output value is interpreted and applied
     * to the motor. The supported control modes include:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT CURRENT} - Achieve a specific current.
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE VOLTAGE} - Achieve a specific voltage.
     *   <li>{@link MotorProperties.ControlMode#PERCENTAGE_OUTPUT PERCENTAGE_OUTPUT} - Achieve a specific duty cycle percentage.
     *   <li>{@link MotorProperties.ControlMode#POSITION POSITION} - Achieve a specific position using advanced control.
     *   <li>{@link MotorProperties.ControlMode#VELOCITY VELOCITY} - Achieve a specific velocity using advanced control.
     * </ul>
     * </p>
     *
     * <p>For {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes,
     * a trapezoidal motion profile can optionally be used. To enable it, ensure both {@link MotorConfiguration#profiledMaxVelocity profiledMaxVelocity}
     * and {@link MotorConfiguration#profiledTargetAcceleration profiledTargetAcceleration} are set.
     * </p>
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value
     */
    void setOutput(MotorProperties.ControlMode controlMode, double output);


    /**
     * Sets the output of the motor based on the specified control mode, desired output value, and custom feedforward.
     *
     * <p>This method utilizes the built-in feedforward and PID controller to achieve precise control
     * over the motor. The control mode determines how the output value is interpreted and applied
     * to the motor. The supported control modes include:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT CURRENT} - Achieve a specific current.
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE VOLTAGE} - Achieve a specific voltage.
     *   <li>{@link MotorProperties.ControlMode#PERCENTAGE_OUTPUT PERCENTAGE_OUTPUT} - Achieve a specific duty cycle percentage.
     *   <li>{@link MotorProperties.ControlMode#POSITION POSITION} - Achieve a specific position using advanced control.
     *   <li>{@link MotorProperties.ControlMode#VELOCITY VELOCITY} - Achieve a specific velocity using advanced control.
     * </ul>
     * </p>
     *
     * <p>For {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes,
     * a trapezoidal motion profile can optionally be used. To enable it, ensure both {@link MotorConfiguration#profiledMaxVelocity profiledMaxVelocity}
     * and {@link MotorConfiguration#profiledTargetAcceleration profiledTargetAcceleration} are set.
     * </p>
     *
     * <p>The custom feedforward is used to provide additional control over the motor output, allowing for fine-tuned
     * performance. Feedforward is applied only in {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes.
     * </p>
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value (amperes for {@link MotorProperties.ControlMode#CURRENT CURRENT}, volts for {@link MotorProperties.ControlMode#VOLTAGE VOLTAGE},
     *                    percentage for {@link MotorProperties.ControlMode#PERCENTAGE_OUTPUT PERCENTAGE_OUTPUT}, rotations for {@link MotorProperties.ControlMode#POSITION POSITION}
     *                    or rotations per second for {@link MotorProperties.ControlMode#VELOCITY VELOCITY})
     * @param feedforward the custom feedforward to be applied to the motor output
     */
    void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward);


    /**
     * Set the idle mode of the motor
     *
     * @param idleMode The new idle mode
     */
    void setIdleMode(MotorProperties.IdleMode idleMode);

    /**
     * Stop the motor
     */
    void stopMotor();

    /**
     * Sets the encoder position of the motor to a specified value.
     *
     * <p>This method allows for manually setting the encoder position of the motor.
     * This can be useful for resetting the encoder position to a known reference point
     * or for calibrating the motor position in applications that require precise positional control.
     * </p>
     *
     * @param position the desired encoder position to set, in rotations.
     */
    void setMotorEncoderPosition(double position);

    /**
     * Get the ID of the motor
     *
     * @return The ID of the motor
     */
    int getDeviceID();

    /**
     * Retrieves the current position of the motor without any gearing applied.
     *
     * <p>This method returns the position of the motor as measured by the encoder,
     * without taking into account any gearing reductions or multipliers.
     * </p>
     *
     * @return the current position of the motor in rotations
     */
    double getMotorPosition();

    /**
     * Retrieves the current velocity of the motor, with no gearing applied.
     *
     * <p>This method returns the velocity of the motor as measured by the encoder,
     * without taking into account any gearing reductions or multipliers.
     * </p>
     *
     * @return the current velocity of the motor, in rotations per second (RPS).
     */
    double getMotorVelocity();

    /**
     * Get the current running through the motor (STATOR current)
     *
     * @Units - In amps
     */
    double getCurrent();

    /**
     * Get the voltage running through the motor
     *
     * @Units - In volts
     */
    double getVoltage();

    /**
     * Get the current target of the closed-loop PID
     */
    double getClosedLoopTarget();

    /**
     * Get the temperature of the motor
     *
     * @Units - In celsius
     */
    double getTemperature();

    /**
     * Gearing applied
     *
     * @Units - In rotations
     */
    double getSystemPosition();

    /**
     * Gearing applied
     *
     * @Units - In rotations per second
     */
    double getSystemVelocity();

    void setFollowerOf(int masterPort);

    void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz);

    void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes);

    /**
     * Get the raw StatusSignal of the motor. DO NOT USE if not necessary.
     */
    StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType);

    /**
     * Refreshes all status signals.
     * This has the same effect as calling {@link com.ctre.phoenix6.BaseStatusSignal#refreshAll(BaseStatusSignal...)}.
     * DO NOT USE if not necessary.
     */
    void refreshStatusSignals(Properties.SignalType... signalTypes);

    TalonFXSimState getSimulationState();

    boolean configure(MotorConfiguration configuration);

    default MotorProperties.Slot getSlot(int slotToUse, MotorConfiguration currentConfiguration) {
        switch (slotToUse) {
            case 0 -> {
                return currentConfiguration.slot0;
            }
            case 1 -> {
                return currentConfiguration.slot1;
            }
            case 2 -> {
                return currentConfiguration.slot2;
            }
        }

        return currentConfiguration.slot0;
    }

    default boolean isAtSetpoint() {
        if (getCurrentConfiguration().closedLoopTolerance == 0)
            throw new UnsupportedOperationException("Must set closed loop tolerance!");

        return Math.abs(getClosedLoopTarget() - getSystemPosition()) < getCurrentConfiguration().closedLoopTolerance;
    }
}
