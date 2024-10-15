package frc.lib.ruckig;

import static frc.lib.ruckig.Result.Finished;
import static frc.lib.ruckig.Result.Working;

//! Main interface for the Ruckig algorithm
public class Ruckig {
    //! Current input, only for comparison for recalculation
    private InputParameter current_input;

    //! Flag that indicates if the current_input was properly initialized
    private boolean current_input_initialized = false;
    
    //! Calculator for new trajectories
    public TargetCalculator calculator = new TargetCalculator();

    //! Time step between updates (cycle time) in [s]
    public double delta_time;

    public Ruckig( double delta_time) {
        this.delta_time = delta_time ;
    }

    //! Reset the instance (e.g. to force a new calculation in the next update)
    public void reset() {
        current_input_initialized = false;
    }

    //! Calculate a new trajectory for the given input and check for interruption
    private UpdateResult calculate(InputParameter input, OutputParameter output) {
        return calculator.calculate(input, output);
    }

    //! Get the next output state (with step delta_time) along the calculated trajectory for the given input
    public UpdateResult update(InputParameter input, OutputParameter output) {
        if (!current_input_initialized || !input.isEqual(current_input)) {
            UpdateResult calculate_result = calculate(input, output);

            output.trajectory = calculate_result.output_parameter.trajectory;

            current_input = calculate_result.input_parameter;
            current_input_initialized = true;
            output.time = 0.0;
        }

        output.time += delta_time;
        double[] newResults = output.trajectory.at_time(
                output.time);

        output.new_position = newResults[0];
        output.new_velocity = newResults[1];
        output.new_acceleration = newResults[2];
        output.new_jerk = newResults[3];

        current_input = output.update_input_from_output(current_input);

        if (output.time > output.trajectory.get_duration()) {
            return new UpdateResult(current_input, output, Finished);
        }

        return new UpdateResult(current_input, output, Working);
    }
}
