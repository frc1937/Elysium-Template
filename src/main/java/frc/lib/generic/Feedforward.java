package frc.lib.generic;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class Feedforward {
    private final double kS; //Volts required to overcome the force of static friction
    private final double kV; //Volts required to maintain a velocity of one unit
    private final double kA; //Volts required to accelerate the mechanism by one unit
    private final double kG; //Volts required to overcome the force of gravity

    private final Properties.FeedforwardType type;

    private final ArmFeedforward armFeedforward;
    private final ElevatorFeedforward elevatorFeedforward;
    private final SimpleMotorFeedforward simpleFeedforward;

    public Feedforward(Properties.FeedforwardType type, double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = 0;

        this.type = type;

        armFeedforward = null;
        elevatorFeedforward = null;
        simpleFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public Feedforward(Properties.FeedforwardType type, double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;

        this.type = type;

        armFeedforward = new ArmFeedforward(kS, kV, kA, kG);
        elevatorFeedforward = new ElevatorFeedforward(kS, kV, kA, kG);
        simpleFeedforward = null;
    }

    /**
     * @param positionRotations     - For arm ONLY. This should be in rotations. For non-arms, this will not be used.
     * @param velocityRotationsPerSec     - unit-less, preferably in rotations per second
     * @param accelerationRotPerSecSquared - unit-less, preferably rotations per second squared
     * @return - the input for the motors
     */
    public double calculate(double positionRotations, double velocityRotationsPerSec, double accelerationRotPerSecSquared) {
        switch (type) {
            case ARM -> {
                exitIfNotPresent(armFeedforward);
                return armFeedforward.calculate(Units.rotationsToRadians(positionRotations), velocityRotationsPerSec, accelerationRotPerSecSquared);
            }

            case ELEVATOR -> {
                exitIfNotPresent(elevatorFeedforward);
                return elevatorFeedforward.calculate(velocityRotationsPerSec, accelerationRotPerSecSquared);
            }

            case SIMPLE -> {
                exitIfNotPresent(simpleFeedforward);
                return simpleFeedforward.calculate(velocityRotationsPerSec, accelerationRotPerSecSquared);
            }
        }

        throw new UnsupportedOperationException("Can't seem to have a type. How is this even possible?");
    }

    /**
     * For both simple motor and elevator feed forwards. No position is required.
     *
     * @param velocity     - unit-less, preferably in rotations per second
     * @param acceleration - unit-less, preferably rotations per second squared
     * @return - the input for the motors
     */
    public double calculate(double velocity, double acceleration) {
        return calculate(0, velocity, acceleration);
    }

    /**
     * For both simple motor and elevator feed forwards. With no position or acceleration
     *
     * @param velocity     - unit-less, preferably in rotations per second
     * @return - the input for the motors
     */
    public double calculate(double velocity) {
        return calculate(0, velocity, 0);
    }

    private void exitIfNotPresent(Object feedforward) {
        if (feedforward == null)
            throw new UnsupportedOperationException("Can't use feedforward with " + type.name() + " type");
    }
}
