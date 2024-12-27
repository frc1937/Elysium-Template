package frc.lib.generic.hardware.motor.hardware.simulations;

/**
 * An abstract class to simulate the physics of a motor.
 */
public abstract class GenericPhysicsSimulation {
    private final double gearRatio;

    protected GenericPhysicsSimulation(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    public double getMotorPositionRotations() {
        return getSystemPositionRotations() * gearRatio;
    }

    public double getMotorVelocityRotationsPerSecond() {
        return getSystemVelocityRotationsPerSecond() * gearRatio;
    }

    public abstract double getCurrent();

    public abstract double getSystemPositionRotations();

    public abstract double getSystemVelocityRotationsPerSecond();

    public abstract double getSystemAccelerationRotationsPerSecondSquared();

    public abstract void setInputVoltage(double voltage);

    public abstract void updateMotor();
}