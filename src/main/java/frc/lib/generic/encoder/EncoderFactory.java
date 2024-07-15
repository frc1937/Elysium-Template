package frc.lib.generic.encoder;

import frc.lib.generic.encoder.hardware.GenericCanCoder;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class EncoderFactory {
    public static Encoder createCanCoder(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
            return new GenericCanCoder(name, port);
        }

        return new Encoder(name);
    }
}
