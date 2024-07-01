package frc.robot.subsystems.kicker.real;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealKickerConstants {
    public static final DigitalInput BEAM_BREAKER = new DigitalInput(0);
    public static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(8);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MOTOR.configFactoryDefault();
        MOTOR.setNeutralMode(NeutralMode.Brake);
    }
}
