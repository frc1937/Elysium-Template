package frc.robot.subsystems.arm.real;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.EncoderSignal;
import frc.lib.generic.encoder.hardware.GenericCanCoder;
import frc.lib.generic.motor.*;
import frc.lib.generic.motor.hardware.GenericSpark;

import static frc.lib.generic.motor.MotorSignal.SignalType.*;
import static frc.robot.subsystems.arm.ArmConstants.TOLERANCE_ROTATIONS;

public class RealArmConstants {
    static final Motor ARM_MOTOR = new GenericSpark("ARM_MOTOR", 1, MotorProperties.SparkType.FLEX);
    public static final Encoder ABSOLUTE_ARM_ENCODER = new GenericCanCoder("ARM_ENCODER", 22);

    static final double PITCH_GEAR_RATIO = 149;

    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);

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

        ABSOLUTE_ARM_ENCODER.setSignalUpdateFrequency(new EncoderSignal(EncoderSignal.SignalType.POSITION));
        ABSOLUTE_ARM_ENCODER.setSignalUpdateFrequency(new EncoderSignal(EncoderSignal.SignalType.VELOCITY));
    }

    private static void configureMotor() {
        MotorConfiguration motorConfiguration = new MotorConfiguration();

        motorConfiguration.gearRatio = PITCH_GEAR_RATIO;

        motorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        motorConfiguration.supplyCurrentLimit = 40;

        motorConfiguration.profiledTargetAcceleration = 0.35;
        motorConfiguration.profiledMaxVelocity = 0.5;

        motorConfiguration.closedLoopTolerance = TOLERANCE_ROTATIONS;

        motorConfiguration.slot0 = new MotorProperties.Slot(
                        0, 0, 0,
                        15.625, 0.85843, 0.097736, 0.21308,
                        GravityTypeValue.Arm_Cosine
        );

        ARM_MOTOR.configure(motorConfiguration);

        ARM_MOTOR.setupSignalUpdates(new MotorSignal(POSITION));
        ARM_MOTOR.setupSignalUpdates(new MotorSignal(VELOCITY));
        ARM_MOTOR.setupSignalUpdates(new MotorSignal(VOLTAGE));

        ARM_MOTOR.setExternalPositionSupplier(ABSOLUTE_ARM_ENCODER::getEncoderPosition);
        ARM_MOTOR.setExternalVelocitySupplier(ABSOLUTE_ARM_ENCODER::getEncoderVelocity);
    }
}
