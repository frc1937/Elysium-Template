package frc.robot.subsystems.flywheel.simulation;

import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.FlywheelSimulation;
import frc.lib.generic.simulation.mechanisms.SpeedMechanism2d;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelInputsAutoLogged;

import static frc.robot.subsystems.flywheel.FlywheelConstants.MAXIMUM_VELOCITY_RPM;
import static frc.robot.subsystems.flywheel.FlywheelConstants.TOLERANCE_ROTATIONS_PER_SECONDS;

public class SimulatedFlywheel extends FlywheelIO {
    private final FlywheelSimulation motor;
    private final SpeedMechanism2d speedMechanism2d;

    private final double diameter;

    private double targetRPS = 0;


    public SimulatedFlywheel(FlywheelSimulation motor, String name, boolean inverted, double diameter) {
        super(name);

        speedMechanism2d = new SpeedMechanism2d("Flywheel" + name, MAXIMUM_VELOCITY_RPM / 60, inverted);

        this.motor = motor;
        this.diameter = diameter;
    }

    @Override
    public double getFlywheelDiameter() {
        return diameter;
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public boolean hasReachedTarget() {
        return Math.abs(motor.getVelocityRotationsPerSecond() - targetRPS) < TOLERANCE_ROTATIONS_PER_SECONDS;
    }

    @Override
    public void flywheelPeriodic() {
        speedMechanism2d.updateMechanism(motor.getVelocityRotationsPerSecond());
    }

    @Override
    public void setTargetVelocity(double velocityRotationsPerSecond) {
        targetRPS = velocityRotationsPerSecond;

        motor.setOutput(MotorProperties.ControlMode.VELOCITY, targetRPS);
        speedMechanism2d.setTargetVelocity(targetRPS);
    }

    @Override
    public void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void refreshInputs(FlywheelInputsAutoLogged flywheelInputs) {
        flywheelInputs.temperature = 0;
        flywheelInputs.velocityRotationsPerSecond = motor.getVelocityRotationsPerSecond();
        flywheelInputs.voltage = motor.getVoltage();
    }
}
