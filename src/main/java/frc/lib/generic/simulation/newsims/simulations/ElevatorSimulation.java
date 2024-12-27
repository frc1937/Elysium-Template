package frc.lib.generic.simulation.newsims.simulations;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.simulation.extensions.ExtendedElevatorSim;
import frc.lib.math.Conversions;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;

public class ElevatorSimulation extends GenericPhysicsSimulation {
    private final ExtendedElevatorSim elevatorSimulation;
    private final double retractedHeightMeters;
    private final double diameterMeters;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters, double retractedHeightMeters, double maximumHeightMeters, boolean simulateGravity) {
        super(gearRatio);
        diameterMeters = drumRadiusMeters + drumRadiusMeters;
        this.retractedHeightMeters = retractedHeightMeters;
        elevatorSimulation = new ExtendedElevatorSim(
                gearbox,
                gearRatio,
                carriageMassKilograms,
                drumRadiusMeters,
                retractedHeightMeters,
                maximumHeightMeters,
                simulateGravity,
                retractedHeightMeters
        );
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getSystemPositionRotations() {
        return Conversions.metresToRotations(elevatorSimulation.getPositionMeters() - retractedHeightMeters, diameterMeters);
    }

    @Override
    public double getSystemVelocityRotationsPerSecond() {
        return Conversions.metresToRotations(elevatorSimulation.getVelocityMetersPerSecond(), diameterMeters);
    }

    @Override
    public double getSystemAccelerationRotationsPerSecondSquared() {
        return Conversions.metresToRotations(elevatorSimulation.getAccelerationMetersPerSecondSquared(), diameterMeters);
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
    public void updateMotor() {
        elevatorSimulation.update(ROBOT_PERIODIC_LOOP_TIME);
    }
}