package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class CTREUtil {
    public static void applyConfig(Object device, Object config) {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            if (device instanceof TalonFX talonFX && config instanceof TalonFXConfiguration talonFXConfig) {
                statusCode = talonFX.getConfigurator().apply(talonFXConfig);
            }

            if (device instanceof CANcoder canCoder && config instanceof CANcoderConfiguration canCoderConfig) {
                statusCode = canCoder.getConfigurator().apply(canCoderConfig);
            }

            counter--;
        }
    }
}
