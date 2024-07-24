package frc.lib.generic.hardware.motor;

import frc.lib.generic.hardware.motor.hardware.GenericSpark;
import frc.lib.generic.hardware.motor.hardware.GenericTalonFX;
import frc.lib.generic.hardware.motor.hardware.GenericTalonSRX;
import frc.lib.generic.hardware.motor.hardware.SimulatedMotor;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class MotorFactory {
    public static Motor createSpark(String name, int port, MotorProperties.SparkType type) {
        Motor motor = createSimOrReplayMotor(name);
        if (motor != null) return motor;
        return new GenericSpark(name, port, type);
    }

    public static Motor createTalonFX(String name, int port) {
        Motor motor = createSimOrReplayMotor(name);
        if (motor != null) return motor;
        return new GenericTalonFX(name, port);
    }

    public static Motor createTalonSRX(String name, int port) {
        Motor motor = createSimOrReplayMotor(name);
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
