package frc.robot.subsystems.kicker.real;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.generic.motor.GenericTalonSRX;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

public class RealKickerConstants {
    public static final DigitalInput BEAM_BREAKER = new DigitalInput(0);
    public static final Motor MOTOR = new GenericTalonSRX(8);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;

        MOTOR.configure(configuration);
    }
}
