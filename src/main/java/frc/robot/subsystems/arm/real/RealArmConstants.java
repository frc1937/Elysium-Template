package frc.robot.subsystems.arm.real;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.Properties;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.GenericCanCoder;
import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

public class RealArmConstants {
    static final Motor ARM_MOTOR = new GenericSpark(1, MotorProperties.SparkType.FLEX);
    static final Encoder ABSOLUTE_ARM_ENCODER = new GenericCanCoder(22);

    static final double PITCH_GEAR_RATIO = 1.0 / 149;

    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);

    static final double
            PITCH_KS = 0,
            PITCH_KV = 14,
            PITCH_KA = 1.2632,
            PITCH_KG = 0.23,
            PITCH_KP = 40,
            PITCH_KI = 0.0,
            PITCH_KD = 0.0,
            PITCH_MAX_VELOCITY = 0.5,
            PITCH_MAX_ACCELERATION = 0.5;

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureEncoder() {
        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = true;
        encoderConfiguration.offsetRotations = PIVOT_ENCODER_OFFSET.getRotations();
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.NegativeHalfToHalf;

        ABSOLUTE_ARM_ENCODER.configure(encoderConfiguration);

        ABSOLUTE_ARM_ENCODER.setSignalUpdateFrequency(Properties.SignalType.POSITION, 50);
    }

    private static void configureMotor() {
        MotorConfiguration motorConfiguration = new MotorConfiguration();

        motorConfiguration.gearRatio = PITCH_GEAR_RATIO;

        motorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        motorConfiguration.supplyCurrentLimit = 40;

        motorConfiguration.slot0 = new MotorProperties.Slot(
                1, 0, 0,
                0, 0, 0, 0.1,
                GravityTypeValue.Arm_Cosine
//                PITCH_KP,
//                PITCH_KI,
//                PITCH_KD,
//                PITCH_KV,
//                PITCH_KA,
//                PITCH_KS,
//                PITCH_KG,
//                GravityTypeValue.Arm_Cosine
        );

        ARM_MOTOR.configure(motorConfiguration);
    }
}
