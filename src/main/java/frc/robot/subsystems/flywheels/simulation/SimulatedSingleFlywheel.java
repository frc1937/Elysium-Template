package frc.robot.subsystems.flywheels.simulation;

import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.FlywheelSimulation;
import frc.lib.generic.simulation.mechanisms.SpeedMechanism2d;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.flywheels.SingleFlywheelIO;
import frc.robot.subsystems.flywheels.SingleFlywheelInputsAutoLogged;

public class SimulatedSingleFlywheel extends SingleFlywheelIO {
    private final FlywheelSimulation motor;
    private final SpeedMechanism2d speedMechanism2d;
    private final double flywheelDiameter;

    private double targetRPS = 0;

    public SimulatedSingleFlywheel(String name, FlywheelSimulation motor, double flywheelDiameter, boolean inverted) {
        super(name);

        this.motor = motor;
        this.flywheelDiameter = flywheelDiameter;

        speedMechanism2d = new SpeedMechanism2d("Flywheel" + name, FlywheelsConstants.MAXIMUM_VELOCITY_RPM / 60, inverted);
    }

    @Override
    protected void setTargetVelocityRPS(double velocityRPS) {
        targetRPS = velocityRPS;

        motor.setOutput(MotorProperties.ControlMode.VELOCITY, targetRPS);
        speedMechanism2d.setTargetVelocity(targetRPS);
    }

    @Override
    protected void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    protected double getFlywheelDiameter() {
        return flywheelDiameter;
    }

    @Override
    protected void stop() {
        motor.stop();
    }

    @Override
    protected void flywheelPeriodic() {
        speedMechanism2d.updateMechanism(motor.getVelocityRotationsPerSecond());
    }

    @Override
    protected void refreshInputs(SingleFlywheelInputsAutoLogged singleFlywheelInputs) {
        singleFlywheelInputs.velocityRotationsPerSecond = motor.getVelocityRotationsPerSecond();
        singleFlywheelInputs.voltage = motor.getVoltage();
        singleFlywheelInputs.temperature = 0;
        singleFlywheelInputs.targetVelocityRotationsPerSecond = targetRPS;
    }
}
