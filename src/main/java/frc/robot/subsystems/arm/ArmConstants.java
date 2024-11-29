package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import frc.lib.util.LoggedTunableNumber;

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
            Seconds.of(7)
    );

    static final double TOLERANCE_ROTATIONS = Units.degreesToRotations(1.1);

    static final SingleJointedArmMechanism2d ARM_MECHANISM =
            new SingleJointedArmMechanism2d("ArmMechanism", new Color8Bit(Color.kRed));

    static final Motor ARM_MOTOR = MotorFactory.createSpark("Arm",62, MotorProperties.SparkType.FLEX);
    static final Encoder ABSOLUTE_ARM_ENCODER = EncoderFactory.createCanCoder("Arm Encoder", 22);

    static final double PITCH_GEAR_RATIO = 149;

    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);

    public static final LoggedTunableNumber CURRENT_ANGLE_CALIB = new LoggedTunableNumber("CalibratedShootingAngle", 65);
    public static final InterpolatingDoubleTreeMap SHOOTING_ANGLES = new InterpolatingDoubleTreeMap();

    static {
        configureMotor();
        configureEncoder();
        configureShootingAngles();
    }

    private static void configureShootingAngles() {
        SHOOTING_ANGLES.put(0.0, 57.295);
        SHOOTING_ANGLES.put(0.2, 57.295);
        SHOOTING_ANGLES.put(0.4, 57.281);
        SHOOTING_ANGLES.put(0.6, 57.012);
        SHOOTING_ANGLES.put(0.8, 56.036);
        SHOOTING_ANGLES.put(1.0, 54.247);
        SHOOTING_ANGLES.put(1.2, 51.861);
        SHOOTING_ANGLES.put(1.4, 49.159);
        SHOOTING_ANGLES.put(1.6, 46.369);
        SHOOTING_ANGLES.put(1.8, 43.636);
        SHOOTING_ANGLES.put(2.0, 41.04);
        SHOOTING_ANGLES.put(2.2, 38.621);
        SHOOTING_ANGLES.put(2.4, 36.391);
        SHOOTING_ANGLES.put(2.6, 34.346);
        SHOOTING_ANGLES.put(2.8, 32.477);
        SHOOTING_ANGLES.put(3.0, 30.77);
        SHOOTING_ANGLES.put(3.2, 29.211);
        SHOOTING_ANGLES.put(3.4, 27.784);
        SHOOTING_ANGLES.put(3.6, 26.477);
        SHOOTING_ANGLES.put(3.8, 25.277);
        SHOOTING_ANGLES.put(4.0, 24.173);
        SHOOTING_ANGLES.put(4.2, 23.154);
        SHOOTING_ANGLES.put(4.4, 22.213);
        SHOOTING_ANGLES.put(4.6, 21.341);
        SHOOTING_ANGLES.put(4.8, 20.532);
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
        final MotorConfiguration motorConfiguration = new MotorConfiguration();

        motorConfiguration.gearRatio = PITCH_GEAR_RATIO;

        motorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        motorConfiguration.supplyCurrentLimit = 40;

        motorConfiguration.profiledMaxVelocity = 0.6;
        motorConfiguration.profiledTargetAcceleration = 0.4;
        motorConfiguration.profiledJerk = 4;

        motorConfiguration.closedLoopTolerance = TOLERANCE_ROTATIONS;

        motorConfiguration.slot0 = new MotorProperties.Slot(
                5, 0, 0,
                15.625, 0.85843, 0.1205, 0.21308,
                MotorProperties.GravityType.ARM
        );

        motorConfiguration.closedLoopTolerance = TOLERANCE_ROTATIONS;

        motorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500Foc(2),
                250,
                0.3,
                0.5,
                Rotation2d.fromDegrees(-10),
                Rotation2d.fromDegrees(150),
                true
        );

        motorConfiguration.simulationSlot = new MotorProperties.Slot(
                150,
                0,
                0,
                29.887,
                0,
                0.1,
                0.32,
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
