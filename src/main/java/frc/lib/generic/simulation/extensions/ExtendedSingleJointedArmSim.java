package frc.lib.generic.simulation.extensions;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ExtendedSingleJointedArmSim extends SingleJointedArmSim {
    public ExtendedSingleJointedArmSim(LinearSystem<N2, N1, N1> plant, DCMotor gearbox, double gearing, double armLengthMeters, double minAngleRads, double maxAngleRads, boolean simulateGravity, double startingAngleRads, Matrix<N1, N1> measurementStdDevs) {
        super(plant, gearbox, gearing, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads, measurementStdDevs);
    }

    public ExtendedSingleJointedArmSim(LinearSystem<N2, N1, N1> plant, DCMotor gearbox, double gearing, double armLengthMeters, double minAngleRads, double maxAngleRads, boolean simulateGravity, double startingAngleRads) {
        super(plant, gearbox, gearing, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads);
    }

    public ExtendedSingleJointedArmSim(DCMotor gearbox, double gearing, double jKgMetersSquared, double armLengthMeters, double minAngleRads, double maxAngleRads, boolean simulateGravity, double startingAngleRads) {
        super(gearbox, gearing, jKgMetersSquared, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads);
    }

    public ExtendedSingleJointedArmSim(DCMotor gearbox, double gearing, double jKgMetersSquared, double armLengthMeters, double minAngleRads, double maxAngleRads, boolean simulateGravity, double startingAngleRads, Matrix<N1, N1> measurementStdDevs) {
        super(gearbox, gearing, jKgMetersSquared, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads, measurementStdDevs);
    }

    /**
     * Returns the acceleration of the arm.
     *
     * @return The acceleration of the arm in radians per second squared.
     */
    public double getAccelerationRadiansPerSecondSquared() {
        // Get the current state vector (position and velocity)
        Matrix<N2, N1> x = m_x;

        // Get the input voltage
        Matrix<N1, N1> u = m_u;

        // Compute the state derivative (xdot = A * x + B * u)
        Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(u));

        // The acceleration is the second element of the state derivative vector
        return xdot.get(1, 0);
    }
}
