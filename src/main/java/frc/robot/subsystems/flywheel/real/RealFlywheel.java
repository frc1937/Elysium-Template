package frc.robot.subsystems.flywheel.real;

import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.flywheel.FlywheelConstants.TOLERANCE_ROTATIONS_PER_SECONDS;

public class RealFlywheel extends FlywheelIO {
    private final Motor motor;
    private final double diameter;

    public RealFlywheel(Motor motor, String name, double diameter) {
        super(name);

        this.motor = motor;
        this.diameter = diameter;
    }

    @Override
    public double getFlywheelDiameter() {
        return diameter;
    }

    @Override
    public void setTargetVelocity(double velocityRotationsPerSecond) {
        Logger.recordOutput("FLywheel" + getName(), velocityRotationsPerSecond);
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, velocityRotationsPerSecond);
    }

    @Override
    public void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public boolean hasReachedTarget() {
        return Math.abs(motor.getClosedLoopTarget() - motor.getSystemPosition()) < TOLERANCE_ROTATIONS_PER_SECONDS;
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void refreshInputs(FlywheelInputsAutoLogged flywheelInputs) {
        flywheelInputs.temperature = motor.getTemperature() ;
        flywheelInputs.velocityRotationsPerSecond = motor.getSystemVelocity();
        flywheelInputs.voltage = motor.getVoltage();
    }
}
