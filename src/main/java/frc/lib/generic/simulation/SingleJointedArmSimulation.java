package frc.lib.generic.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static frc.lib.generic.simulation.SimulationConstants.ROBORIO_LOOP_TIME;

public class SingleJointedArmSimulation extends GenericSimulation {
    private final SingleJointedArmSim armSimulation;

    public SingleJointedArmSimulation(DCMotor gearbox, double gearRatio, double armLengthMeters, double armMassKilograms, Rotation2d minimumAngle, Rotation2d maximumAngle, boolean simulateGravity) {
        armSimulation = new SingleJointedArmSim(
                gearbox,
                gearRatio,
                SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKilograms),
                armLengthMeters,
                minimumAngle.getRadians(),
                maximumAngle.getRadians(),
                simulateGravity,
                minimumAngle.getRadians()
        );
    }

    @Override
    public double getCurrent() {
        return armSimulation.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        armSimulation.setInputVoltage(voltage);
    }

    @Override
    public void update() {
        armSimulation.update(ROBORIO_LOOP_TIME);
    }

    @Override
    public double getPositionRotations() {
        return Units.radiansToRotations(armSimulation.getAngleRads());
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return Units.radiansToRotations(armSimulation.getVelocityRadPerSec());
    }
}
