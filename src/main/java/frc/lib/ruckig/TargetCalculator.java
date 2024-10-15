package frc.lib.ruckig;

import static frc.lib.ruckig.Result.Working;
import static frc.lib.ruckig.Utilities.back;

public class TargetCalculator {
    public UpdateResult calculate(InputParameter input, OutputParameter output_parameter) {
        output_parameter.trajectory.profile.set_boundary(input.current_position, input.current_velocity,
                input.current_acceleration, input.target_position);

        PositionThirdOrderStep1 step1 = new PositionThirdOrderStep1(
                output_parameter.trajectory.profile.p[0], output_parameter.trajectory.profile.v[0],
                output_parameter.trajectory.profile.a[0],

                output_parameter.trajectory.profile.pf, input.max_velocity, -input.max_velocity,
                input.max_acceleration, -input.max_acceleration, input.max_jerk
        );

        output_parameter.trajectory.profile = step1.get_profile(output_parameter.trajectory.profile);


        if (back(output_parameter.trajectory.profile.totalTime) > output_parameter.trajectory.duration)
            output_parameter.trajectory.duration = back(output_parameter.trajectory.profile.totalTime);

        return new UpdateResult(input, output_parameter, Working);
    }
}

