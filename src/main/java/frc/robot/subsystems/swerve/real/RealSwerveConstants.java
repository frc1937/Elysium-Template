package frc.robot.subsystems.swerve.real;

import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

public class RealSwerveConstants {
    static final Motor FL_STEER_MOTOR = new GenericSpark(0, MotorProperties.SparkType.MAX);

    static final MotorConfiguration steerMotorConfiguration = new MotorConfiguration();

    static {
        setupSteerMotorConfiguration();
    }

    private static void setupSteerMotorConfiguration() {

        steerMotorConfiguration.slot0 = new MotorProperties.Slot(1, 0, 0, 1, 0, 0);

    }
}
