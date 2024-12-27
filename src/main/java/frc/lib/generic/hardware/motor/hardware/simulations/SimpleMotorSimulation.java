package frc.lib.generic.hardware.motor.hardware.simulations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.simulation.extensions.ExtendedDCMotorSim;

public class SimpleMotorSimulation extends GenericPhysicsSimulation {
    private final ExtendedDCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        super(gearRatio);
        motorSimulation = new ExtendedDCMotorSim(gearbox, momentOfInertia, gearRatio);
    }

    public SimpleMotorSimulation(DCMotor gearbox, double gearRatio, double kv, double ka) {
        super(gearRatio);
        motorSimulation = new ExtendedDCMotorSim(LinearSystemId.createDCMotorSystem(kv, ka), gearbox, gearRatio);
    }


    @Override
    public double getCurrent() {
        return motorSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getSystemPositionRotations() {
        return motorSimulation.getAngularPositionRotations();
    }

    @Override
    public double getSystemVelocityRotationsPerSecond() {
        return motorSimulation.getAngularVelocityRPM() / 60;
    }

    @Override
    public double getSystemAccelerationRotationsPerSecondSquared() {
        return Units.radiansToRotations(motorSimulation.getAccelerationRadiansPerSecondSquared());
    }

    @Override
    public void setInputVoltage(double voltage) {
        motorSimulation.setInputVoltage(voltage);
    }

    @Override
    public void updateMotor() {
        motorSimulation.update(0.02);
    }
}