package frc.lib.generic.motor;

import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.Properties;

public interface Motor {
    void setP(double kP, int slot);

    void setOutput(MotorProperties.ControlMode controlMode, double output);
    void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward);

    void setIdleMode(MotorProperties.IdleMode idleMode);

    void stopMotor();

    void setMotorPosition(double position);

    int getDeviceID();

    /** No gearing applied*/
    double getMotorPosition();
    /** No gearing applied*/
    double getMotorVelocity();

    /** Get the current running through the motor (SUPPLY current)*/
    double getCurrent();

    /** Get the voltage running through the motor */
    double getVoltage();

    /** Get the current target of the closed-loop PID*/
    double getClosedLoopTarget();

    /** Get the temperature of the motor, in Celsius */
    double getTemperature();

    /** Gearing applied*/
    double getSystemPosition();
    /** Gearing applied*/
    double getSystemVelocity();

    void setFollowerOf(int masterPort);

    void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz);
    void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes);

    TalonFXSimState getSimulationState();

    boolean configure(MotorConfiguration configuration);
}
