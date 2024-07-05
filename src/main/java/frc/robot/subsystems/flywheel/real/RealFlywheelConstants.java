package frc.robot.subsystems.flywheel.real;

import frc.lib.generic.Properties;
import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class RealFlywheelConstants extends FlywheelConstants {
    private static final Motor LEFT_FLYWHEEL_MOTOR = new GenericSpark(16, MotorProperties.SparkType.FLEX);
    private static final Motor RIGHT_FLYWHEEL_MOTOR = new GenericSpark(15, MotorProperties.SparkType.FLEX);

    private static final MotorProperties.Slot LEFT_SLOT =
            new MotorProperties.Slot(0.003, 0.0, 0.0, 0.0969743, 0.0, 0.02426);
    private static final MotorProperties.Slot RIGHT_SLOT =
            new MotorProperties.Slot(0.003, 0.0, 0.0, 0.097728, 0.0, 0.022648);

    private static final MotorConfiguration configuration = new MotorConfiguration();

    static {
        configureMotor(LEFT_FLYWHEEL_MOTOR, true, LEFT_SLOT);
        configureMotor(RIGHT_FLYWHEEL_MOTOR, RIGHT_MOTOR_INVERT, RIGHT_SLOT);
    }

    private static void configureMotor(Motor motor, boolean invert, MotorProperties.Slot slot) {
        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 80;
        configuration.statorCurrentLimit = 100;

        configuration.slot0 = new MotorProperties.Slot(0.001, 0, 0, 0, 0, 0);// slot;

        motor.configure(configuration);

        motor.setSignalUpdateFrequency(Properties.SignalType.CLOSED_LOOP_TARGET, 50);
        motor.setSignalUpdateFrequency(Properties.SignalType.VELOCITY, 50);
        motor.setSignalUpdateFrequency(Properties.SignalType.TEMPERATURE, 50);
        motor.setSignalUpdateFrequency(Properties.SignalType.VOLTAGE, 50);
    }

    public static FlywheelIO[] getFlywheels() {
        return new RealFlywheel[]{
                new RealFlywheel(LEFT_FLYWHEEL_MOTOR, "LeftReal", LEFT_FLYWHEEL_DIAMETER),
                new RealFlywheel(RIGHT_FLYWHEEL_MOTOR, "RightReal", RIGHT_FLYWHEEL_DIAMETER)
        };
    }
}
