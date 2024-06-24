package frc.lib.generic.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.math.Conversions;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public class ElevatorSimulation extends GenericSimulation {
    private final ElevatorSim elevatorSimulation;

    private final double minimumHeightMetres;
    private final double drumDiameterMetres;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters, double minimumHeightMetres, double maximumHeightMetres, boolean simulateGravity) {
        elevatorSimulation = new ElevatorSim(
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
        return Conversions.metersPerSecondToRotationsPerSecond(elevatorSimulation.getVelocityMetersPerSecond(), drumDiameterMetres);
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
