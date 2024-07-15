package frc.lib.generic.motor;

import frc.lib.generic.motor.hardware.GenericSpark;
import frc.lib.generic.motor.hardware.GenericTalonFX;
import frc.lib.generic.motor.hardware.GenericTalonSRX;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class MotorFactory {
    public static Motor createSpark(String name, int port, MotorProperties.SparkType type) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new Motor(name);
        }
//todo: Incorporate sim
        return new GenericSpark(name, port, type);
    }

    public static Motor createTalonFX(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new Motor(name);
        }

        return new GenericTalonFX(name, port);
    }

    public static Motor createTalonSRX(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new Motor(name);
        }

        return new GenericTalonSRX(name, port);
    }
}
