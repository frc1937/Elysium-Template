package frc.lib.generic.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.simulation.extensions.ExtendedElevatorSim;
import frc.lib.math.Conversions;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public class ElevatorSimulation extends GenericSimulation {
    private final ExtendedElevatorSim elevatorSimulation;

    private final double minimumHeightMetres;
    private final double drumDiameterMetres;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters, double minimumHeightMetres, double maximumHeightMetres, boolean simulateGravity) {
        elevatorSimulation = new ExtendedElevatorSim(
                gearbox,
                gearRatio,
                carriageMassKilograms,
                drumRadiusMeters,
                minimumHeightMetres,
                maximumHeightMetres,
                simulateGravity,
                minimumHeightMetres
        );

        this.minimumHeightMetres = minimumHeightMetres;
        this.drumDiameterMetres = drumRadiusMeters * 2;
    }


    @Override
    public double getPositionRotations() {
        return Conversions.metresToRotations(
                elevatorSimulation.getPositionMeters() - minimumHeightMetres, drumDiameterMetres);
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return Conversions.mpsToRps(elevatorSimulation.getVelocityMetersPerSecond(), drumDiameterMetres);
    }

    @Override
    public double getAccelerationRotationsPerSecondSquared() {
        return Conversions.mpsToRps(elevatorSimulation.getAccelerationMetersPerSecondSquared(), drumDiameterMetres);
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    void setVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
     void update() {
        elevatorSimulation.update(ROBORIO_LOOP_TIME);
    }
}
