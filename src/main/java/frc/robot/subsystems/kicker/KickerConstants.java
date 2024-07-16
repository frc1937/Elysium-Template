package frc.robot.subsystems.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorFactory;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.sensors.Sensor;
import frc.lib.generic.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;

public class KickerConstants {
    public static final Sensor BEAM_BREAKER = SensorFactory.createDigitalInput("Beam Breaker", 0);
    public static final Motor MOTOR = MotorFactory.createTalonSRX("Kicker", 8);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;

        configuration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeo550(1),
                1.0,
                0.003
        );

        MOTOR.configure(configuration);
    }
}