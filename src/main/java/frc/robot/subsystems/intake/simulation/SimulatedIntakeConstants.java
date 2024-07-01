package frc.robot.subsystems.intake.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SimpleMotorSimulation;

public class SimulatedIntakeConstants {
    public static final SimpleMotorSimulation MOTOR = new SimpleMotorSimulation(
            DCMotor.getNeoVortex(1),
            1.0,
            0.003
    );

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration config = new MotorConfiguration();

        config.inverted = true;
        config.idleMode = MotorProperties.IdleMode.COAST;

        MOTOR.configure(config);
    }
}
