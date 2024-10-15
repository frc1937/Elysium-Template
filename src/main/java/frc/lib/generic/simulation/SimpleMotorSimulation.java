package frc.lib.generic.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.simulation.extensions.ExtendedDCMotorSim;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public class SimpleMotorSimulation extends GenericSimulation {
    private final ExtendedDCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        motorSimulation = new ExtendedDCMotorSim(gearbox, gearRatio, momentOfInertia);
    }

    @Override
    public double getCurrent() {
        return motorSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getPositionRotations() {
        return Units.radiansToRotations(motorSimulation.getAngularPositionRad());
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return Units.radiansToRotations(motorSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    public double getAccelerationRotationsPerSecondSquared() {
        return Units.radiansToRotations(motorSimulation.getAccelerationRadiansPerSecondSquared());
    }

    @Override
    void setVoltage(double voltage) {
        motorSimulation.setInputVoltage(voltage);
    }

    @Override
    void update()  {
        motorSimulation.update(ROBORIO_LOOP_TIME);
    }
}
