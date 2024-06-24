package frc.lib.generic.motor;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class MotorProperties {
    public enum IdleMode {
        COAST, BRAKE
    }

    public enum ControlMode {
        CURRENT, POSITION, VELOCITY, VOLTAGE, PERCENTAGE_OUTPUT
    }

    public record Slot(double kP, double kD, double kI, double kV, double kA, double kS, double kG, GravityTypeValue gravityType) { }
}
