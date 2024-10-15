package frc.lib.generic.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.simulation.extensions.ExtendedFlywheelSim;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public class FlywheelSimulation extends GenericSimulation {
    private final ExtendedFlywheelSim flywheelSimulation;
    private double lastPositionRadians = 0;

    public FlywheelSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        flywheelSimulation = new ExtendedFlywheelSim(gearbox, gearRatio, momentOfInertia);
    }

    public FlywheelSimulation(DCMotor gearbox, double gearRatio, double kv, double ka) {
        flywheelSimulation = new ExtendedFlywheelSim(LinearSystemId.identifyVelocitySystem(kv, ka), gearbox, gearRatio);
    }

    @Override
    public double getCurrent() {
        return flywheelSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getPositionRotations() {
        return Units.radiansToRotations(lastPositionRadians);
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return Units.radiansToRotations(flywheelSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    public double getAccelerationRotationsPerSecondSquared() {
        return Units.radiansToRotations(flywheelSimulation.getAccelerationRadiansPerSecondSquared());
    }

    @Override
    void setVoltage(double voltage) {
        flywheelSimulation.setInputVoltage(voltage);
    }

    @Override
    void update() {
        flywheelSimulation.update(ROBORIO_LOOP_TIME);
        lastPositionRadians = lastPositionRadians + flywheelSimulation.getAngularVelocityRadPerSec() * ROBORIO_LOOP_TIME;
    }
}
