package frc.lib.generic.pigeon;

import frc.lib.generic.pigeon.hardware.GenericIMU;
import frc.lib.generic.pigeon.hardware.SimulatedIMU;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class PigeonFactory {
    public static Pigeon createPigeon(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new Pigeon(name);
        }

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedIMU(name);
        }

        return new GenericIMU(name, port);
    }
}
