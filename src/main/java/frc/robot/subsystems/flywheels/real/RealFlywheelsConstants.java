package frc.robot.subsystems.flywheels.real;

import frc.lib.generic.Properties;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.GenericSpark;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.flywheels.SingleFlywheelIO;

import java.util.Optional;

import static frc.robot.subsystems.swerve.SwerveConstants.ofReplayable;

public class RealFlywheelsConstants extends FlywheelsConstants {
    private static final Motor LEFT_FLYWHEEL_MOTOR = new GenericSpark(28, MotorProperties.SparkType.FLEX);
    private static final Motor RIGHT_FLYWHEEL_MOTOR = new GenericSpark(15, MotorProperties.SparkType.FLEX);

//    private static final MotorProperties.Slot LEFT_SLOT =
//            new MotorProperties.Slot(0.003, 0.0, 0.0, 0.0969743, 0.0, 0.02426);
//    private static final MotorProperties.Slot RIGHT_SLOT =
//            new MotorProperties.Slot(0.003, 0.0, 0.0, 0.097728, 0.0, 0.022648);

    private static final MotorProperties.Slot
            LEFT_SLOT = new MotorProperties.Slot(0.0001, 0, 0, 0, 0, 0),
            RIGHT_SLOT = new MotorProperties.Slot(0.0001, 0, 0, 0, 0, 0);

    static {
        configureMotor(LEFT_FLYWHEEL_MOTOR, true, LEFT_SLOT);
        configureMotor(RIGHT_FLYWHEEL_MOTOR, false, RIGHT_SLOT);
    }

    private static void configureMotor(Motor motor, boolean invert, MotorProperties.Slot slot) {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 40;
        configuration.statorCurrentLimit = 40;

        configuration.slot0 = slot;

        int i = 0;

        while (!motor.configure(configuration) && i < 10) {
            i++;
        }

        motor.setSignalUpdateFrequency(Properties.SignalType.CLOSED_LOOP_TARGET, 50);
        motor.setSignalUpdateFrequency(Properties.SignalType.VELOCITY, 50);
        motor.setSignalUpdateFrequency(Properties.SignalType.TEMPERATURE, 50);
        motor.setSignalUpdateFrequency(Properties.SignalType.VOLTAGE, 50);
    }

    @Override
    protected Optional<SingleFlywheelIO[]> getFlywheels() {
        return ofReplayable(() -> new SingleFlywheelIO[]{
                new RealSingleFlywheel("Right", RIGHT_FLYWHEEL_MOTOR, RIGHT_FLYWHEEL_DIAMETER),
                new RealSingleFlywheel("Left", LEFT_FLYWHEEL_MOTOR, LEFT_FLYWHEEL_DIAMETER)
        });
    }
}
