package frc.robot.subsystems.flywheels.real;

import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.flywheels.SingleFlywheelIO;
import frc.robot.subsystems.flywheels.SingleFlywheelInputsAutoLogged;

public class RealSingleFlywheel extends SingleFlywheelIO {
    private final Motor motor;

    public RealSingleFlywheel(String name, Motor motor, double flywheelDiameter) {
        super(name, flywheelDiameter, motor.getCurrentConfiguration().inverted);

        this.motor = motor;
    }

    @Override
    protected void setTargetVelocity(double velocityRPS) {
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, velocityRPS);
    }

    @Override
    protected void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    @Override
    protected void refreshInputs(SingleFlywheelInputsAutoLogged singleFlywheelInputs) {
        singleFlywheelInputs.temperature = motor.getTemperature();
        singleFlywheelInputs.targetVelocityRotationsPerSecond = motor.getClosedLoopTarget();
        singleFlywheelInputs.voltage = motor.getVoltage();
        singleFlywheelInputs.velocityRotationsPerSecond = motor.getSystemVelocity();
    }
}
