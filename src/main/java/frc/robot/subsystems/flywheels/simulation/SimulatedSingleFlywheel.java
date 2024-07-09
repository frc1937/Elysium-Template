package frc.robot.subsystems.flywheels.simulation;

import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.FlywheelSimulation;
import frc.robot.subsystems.flywheels.SingleFlywheelIO;
import frc.robot.subsystems.flywheels.SingleFlywheelInputsAutoLogged;

public class SimulatedSingleFlywheel extends SingleFlywheelIO {
    private final FlywheelSimulation motor;

    private double targetRPS = 0;

    public SimulatedSingleFlywheel(String name, FlywheelSimulation motor, double flywheelDiameter, boolean inverted) {
        super(name, flywheelDiameter, inverted);

        this.motor = motor;
    }

    @Override
    protected void setTargetVelocity(double velocityRPS) {
        targetRPS = velocityRPS;
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, targetRPS);
    }

    @Override
    protected void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    protected void stop() {
        motor.stop();
    }

    @Override
    protected void refreshInputs(SingleFlywheelInputsAutoLogged singleFlywheelInputs) {
        singleFlywheelInputs.velocityRotationsPerSecond = motor.getVelocityRotationsPerSecond();
        singleFlywheelInputs.voltage = motor.getVoltage();
        singleFlywheelInputs.temperature = 0;
        singleFlywheelInputs.targetVelocityRotationsPerSecond = targetRPS;
    }
}
