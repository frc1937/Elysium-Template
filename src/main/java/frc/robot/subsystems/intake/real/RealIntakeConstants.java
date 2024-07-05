package frc.robot.subsystems.intake.real;

import frc.lib.generic.motor.GenericTalonSRX;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

public class RealIntakeConstants {
    protected static final Motor MOTOR = new GenericTalonSRX(5);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = true;

        MOTOR.configure(configuration);
    }
}
