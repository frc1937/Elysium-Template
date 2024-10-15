package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderConfiguration;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.encoder.EncoderProperties;
import frc.lib.generic.hardware.encoder.EncoderSignal;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.simulation.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.hardware.motor.MotorSignal.ACCELERATION;
import static frc.lib.generic.hardware.motor.MotorSignal.CLOSED_LOOP_TARGET;
import static frc.lib.generic.hardware.motor.MotorSignal.POSITION;
import static frc.lib.generic.hardware.motor.MotorSignal.VELOCITY;
import static frc.lib.generic.hardware.motor.MotorSignal.VOLTAGE;

public class ArmConstants {
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(2),
            Seconds.of(10)
    );

    static final double TOLERANCE_ROTATIONS = Units.degreesToRotations(2);

    static final SingleJointedArmMechanism2d ARM_MECHANISM =
            new SingleJointedArmMechanism2d("ArmMechanism", new Color8Bit(Color.kRed));

    static final Motor ARM_MOTOR = MotorFactory.createSpark("Arm",62, MotorProperties.SparkType.FLEX);
    static final Encoder ABSOLUTE_ARM_ENCODER = EncoderFactory.createCanCoder("Arm Encoder", 22);

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

        ABSOLUTE_ARM_ENCODER.setupSignalUpdates(EncoderSignal.POSITION);
        ABSOLUTE_ARM_ENCODER.setupSignalUpdates(EncoderSignal.VELOCITY);

        ABSOLUTE_ARM_ENCODER.setSimulatedEncoderPositionSource(ARM_MOTOR::getSystemPosition);
        ABSOLUTE_ARM_ENCODER.setSimulatedEncoderVelocitySource(ARM_MOTOR::getSystemVelocity);
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
                MotorProperties.GravityType.ARM
        );

        motorConfiguration.closedLoopTolerance = TOLERANCE_ROTATIONS;

        motorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500Foc(2),
                227.77777,
                0.5,
                11,
                Rotation2d.fromDegrees(-10),
                Rotation2d.fromDegrees(150),
                true
        );

        motorConfiguration.simulationSlot = new MotorProperties.Slot(
                150,
                0,
                0,
                32,
                0,
                0.2,
                0.2,
                    MotorProperties.GravityType.ARM
                );

        ARM_MOTOR.configure(motorConfiguration);

        ARM_MOTOR.setupSignalUpdates(POSITION);
        ARM_MOTOR.setupSignalUpdates(VELOCITY);
        ARM_MOTOR.setupSignalUpdates(VOLTAGE);
        ARM_MOTOR.setupSignalUpdates(CLOSED_LOOP_TARGET);
        ARM_MOTOR.setupSignalUpdates(ACCELERATION);

        ARM_MOTOR.setExternalPositionSupplier(ABSOLUTE_ARM_ENCODER::getEncoderPosition);
        ARM_MOTOR.setExternalVelocitySupplier(ABSOLUTE_ARM_ENCODER::getEncoderVelocity);
    }
}
