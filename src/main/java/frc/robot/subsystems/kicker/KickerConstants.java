package frc.robot.subsystems.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;

public class KickerConstants {
    public static final Sensor BEAM_BREAKER = SensorFactory.createDigitalInput("Beam Breaker", 0);
    public static final Motor MOTOR = MotorFactory.createTalonSRX("Kicker Motor", 8);

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

        configuration.simulationSlot = new MotorProperties.Slot(5, 0,0);

        MOTOR.configure(configuration);
    }
}
