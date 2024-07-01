package frc.robot.subsystems.flywheel.real;

import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelInputsAutoLogged;

public class RealFlywheel extends FlywheelIO {
    private final Motor motor;

    public RealFlywheel(Motor motor, String name) {
        super(name);

        this.motor = motor;
    }

    @Override
    public void setTargetVelocity(double velocityRotationsPerSecond) {
        motor.setInput(MotorProperties.ControlMode.VELOCITY, velocityRotationsPerSecond);
    }

    @Override
    public void refreshInputs(FlywheelInputsAutoLogged flywheelInputs) {
        flywheelInputs.temperature = motor.getTemperature() ;
        flywheelInputs.velocityRotationsPerSecond = motor.getSystemVelocity();
        flywheelInputs.voltage = motor.getVoltage();
    }
}
