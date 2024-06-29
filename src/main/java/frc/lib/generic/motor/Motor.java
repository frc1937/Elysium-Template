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
    void setP(double kP, int slot);

    /**
     * Put voltage to the motor according to the mode. Utilize the built-in feedforward and PID controller.
     * If you need to you use something more advanced (E.G TrapezoidalProfile) use {@link Motor#setInput(MotorProperties.ControlMode, double, double)}
     * With a custom feedforward.
     * <p>
     * Feedforward only works for POSITION and VELOCITY control
     *
     * @param controlMode The type of input
     * @param input      The input for the voltage
     */
    void setInput(MotorProperties.ControlMode controlMode, double input);

    /**
     * Put voltage to the motor according to the mode. Use custom feedforward for the motor.
     * Feedforward only works for POSITION and VELOCITY control.
     *
     * @param controlMode - The type of input
     * @param input      - The input for the voltage
     * @param feedforward - The custom feedforward to input the motor
     */
    void setInput(MotorProperties.ControlMode controlMode, double input, double feedforward);

    void setIdleMode(MotorProperties.IdleMode idleMode);

    /**
     * Set the voltage of the motor to 0
     */
    void stopMotor();

    void setMotorEncoderPosition(double position);

    int getDeviceID();

    /**
     * No gearing applied
     *
     * @Units - In rotations
     */
    double getMotorPosition();

    /**
     * No gearing applied
     *
     * @Units - In rotations per second
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
     * This has the same effect as calling {@link com.ctre.phoenix6.BaseStatusSignal#refreshAll(BaseStatusSignal...)}}}.
     * DO NOT USE if not necessary.
     */
    void refreshStatusSignals(Properties.SignalType... signalTypes);

    TalonFXSimState getSimulationState();

    boolean configure(MotorConfiguration configuration);
}
