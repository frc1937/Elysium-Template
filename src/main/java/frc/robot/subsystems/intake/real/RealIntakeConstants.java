package frc.robot.subsystems.intake.real;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class RealIntakeConstants {
    protected static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(5);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MOTOR.configFactoryDefault();

        MOTOR.setNeutralMode(NeutralMode.Coast);
        MOTOR.setInverted(true);
    }
}
