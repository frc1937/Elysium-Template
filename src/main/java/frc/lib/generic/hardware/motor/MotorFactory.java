package frc.lib.generic.hardware.motor;

import frc.lib.generic.hardware.motor.hardware.rev.GenericSparkFlex;
import frc.lib.generic.hardware.motor.hardware.rev.GenericSparkMax;
import frc.lib.generic.hardware.motor.hardware.ctre.GenericTalonFX;
import frc.lib.generic.hardware.motor.hardware.ctre.GenericTalonSRX;
import frc.lib.generic.hardware.motor.hardware.simulated.SimulatedMotor;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class MotorFactory {
    public static Motor createSpark(String name, int port, MotorProperties.SparkType type) {
        final Motor motor = createSimOrReplayMotor(name);

        if (motor != null) return motor;

        if (type == MotorProperties.SparkType.FLEX) return new GenericSparkFlex(name, port);
        else return new GenericSparkMax(name, port);
    }

    public static Motor createTalonFX(String name, int port) {
        final Motor motor = createSimOrReplayMotor(name);
        if (motor != null) return motor;
        return new GenericTalonFX(name, port);
    }

    public static Motor createTalonSRX(String name, int port) {
        final Motor motor = createSimOrReplayMotor(name);
        if (motor != null) return motor;
        return new GenericTalonSRX(name, port);
    }

    private static Motor createSimOrReplayMotor(String name) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new Motor(name);
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulatedMotor(name);

        return null;
    }
}
