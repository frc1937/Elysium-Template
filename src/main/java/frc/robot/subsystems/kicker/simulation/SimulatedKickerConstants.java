package frc.robot.subsystems.kicker.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SimpleMotorSimulation;

public class SimulatedKickerConstants {
    public static final SimpleMotorSimulation MOTOR = new SimpleMotorSimulation(
            DCMotor.getNeo550(1),
            1.0,
            0.003);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;

        MOTOR.configure(configuration);
    }
}
