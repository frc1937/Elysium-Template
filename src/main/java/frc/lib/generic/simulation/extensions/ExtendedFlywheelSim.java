package frc.lib.generic.simulation.extensions;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ExtendedFlywheelSim extends FlywheelSim {
    public ExtendedFlywheelSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double gearing) {
        super(plant, gearbox, gearing);
    }

    public ExtendedFlywheelSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double gearing, Matrix<N1, N1> measurementStdDevs) {
        super(plant, gearbox, gearing, measurementStdDevs);
    }

    public ExtendedFlywheelSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
        super(gearbox, gearing, jKgMetersSquared);
    }

    public ExtendedFlywheelSim(DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N1, N1> measurementStdDevs) {
        super(gearbox, gearing, jKgMetersSquared, measurementStdDevs);
    }

    /**
     * Returns the acceleration of the flywheel.
     *
     * @return The acceleration of the flywheel in radians per second squared.
     */
    public double getAccelerationRadiansPerSecondSquared() {
        // Get the current state vector (position and velocity)
        Matrix<N1, N1> x = m_x;

        // Get the input voltage
        Matrix<N1, N1> u = m_u;

        // Compute the state derivative (xdot = A * x + B * u)
        Matrix<N1, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(u));

        // The acceleration is the first element of the state derivative vector
        return xdot.get(0, 0);
    }
}
