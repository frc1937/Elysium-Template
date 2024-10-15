package frc.lib.ruckig;

public class InputParameter {
    void initialize() {
        current_velocity = 0.0;
        current_acceleration = 0.0;
        max_acceleration = Double.POSITIVE_INFINITY;
        max_jerk = Double.POSITIVE_INFINITY;
    }

    //! Current state
    public double current_position, current_velocity, current_acceleration;

    //! Target state
    public double target_position;

    //! Kinematic constraints
    public double max_velocity, max_acceleration, max_jerk;

    public InputParameter(double current_position, double current_velocity, double current_acceleration, double target_position, double max_velocity, double max_acceleration, double max_jerk) {
        this.current_position = current_position;
        this.current_velocity = current_velocity;
        this.current_acceleration = current_acceleration;
        this.target_position = target_position;
        this.max_velocity = max_velocity;
        this.max_acceleration = max_acceleration;
        this.max_jerk = max_jerk;
    }

    InputParameter() {
        initialize();
    }

    boolean isEqual(InputParameter rhs) {
        return (current_position == rhs.current_position
                && current_velocity == rhs.current_velocity
                && current_acceleration == rhs.current_acceleration
                && target_position == rhs.target_position
                && max_velocity == rhs.max_velocity
                && max_acceleration == rhs.max_acceleration
                && max_jerk == rhs.max_jerk
        );
    }
}
