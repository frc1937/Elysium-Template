package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.EncoderSignal;
import frc.lib.generic.encoder.hardware.GenericCanCoder;
import frc.lib.generic.motor.*;
import frc.lib.generic.motor.hardware.GenericSpark;
import frc.lib.generic.motor.hardware.GenericTalonFX;
import frc.lib.generic.pigeon.hardware.GenericIMU;
import frc.lib.generic.pigeon.Pigeon;
import frc.lib.generic.pigeon.PigeonSignal;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

import java.util.Optional;

import static frc.lib.generic.motor.MotorSignal.SignalType.*;

public class RealSwerveConstants extends SwerveConstants {
    static final MotorConfiguration steerMotorConfiguration = new MotorConfiguration();
    static final MotorConfiguration driveMotorConfiguration = new MotorConfiguration();

    static final MotorProperties.IdleMode ANGLE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;
    static final MotorProperties.IdleMode DRIVE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final double OPEN_LOOP_RAMP = 0.1;
    static final double CLOSED_LOOP_RAMP = 0.0;

    static final boolean CAN_CODER_INVERT = false;
    static final boolean ANGLE_MOTOR_INVERT = true;
    static final boolean DRIVE_MOTOR_INVERT = false;

    static final int ANGLE_CURRENT_LIMIT = 30;
    static final int DRIVE_SUPPLY_CURRENT_LIMIT = 35;
    static final int DRIVE_STATOR_CURRENT_LIMIT = 50;

    static final EncoderSignal STEER_POSITION_SIGNAL = new EncoderSignal(EncoderSignal.SignalType.POSITION, true);
    static final MotorSignal
            DRIVE_POSITION_SIGNAL = new MotorSignal(POSITION, true),
            DRIVE_VELOCITY_SIGNAL = new MotorSignal(VELOCITY, true);

    static final MotorProperties.Slot DRIVE_SLOT = new MotorProperties.Slot(0.053067, 0.0, 0.0,
            0.10861,
            0.023132,
            0.27053);

    protected static final Motor FL_STEER_MOTOR = new GenericSpark("FL_STEER_MOTOR", 11, MotorProperties.SparkType.MAX),
            FR_STEER_MOTOR = new GenericSpark("FR_STEER_MOTOR", 10, MotorProperties.SparkType.MAX),
            RL_STEER_MOTOR = new GenericSpark("RL_STEER_MOTOR", 6, MotorProperties.SparkType.MAX),
            RR_STEER_MOTOR = new GenericSpark("RR_STEER_MOTOR", 9, MotorProperties.SparkType.MAX);

    protected static final Motor FL_DRIVE_MOTOR = new GenericTalonFX("FL_DRIVE_MOTOR", 14),
            FR_DRIVE_MOTOR = new GenericTalonFX("FR_DRIVE_MOTOR", 13),
            RL_DRIVE_MOTOR = new GenericTalonFX("RL_DRIVE_MOTOR", 13),
            RR_DRIVE_MOTOR = new GenericTalonFX("RR_DRIVE_MOTOR", 12);

    protected static final Encoder FL_STEER_ENCODER = new GenericCanCoder("FL_STEER_ENCODER", 18),
            FR_STEER_ENCODER = new GenericCanCoder("FR_STEER_ENCODER", 20),
            RL_STEER_ENCODER = new GenericCanCoder("RL_STEER_ENCODER", 19),
            RR_STEER_ENCODER = new GenericCanCoder("RR_STEER_ENCODER", 21);

    static final double[] STEER_ENCODER_OFFSET = {0.677246, 0.282715, 0.533447, 0.313721};

    static final Encoder[] STEER_ENCODER = {FL_STEER_ENCODER, FR_STEER_ENCODER, RL_STEER_ENCODER, RR_STEER_ENCODER};
    static final Motor[] STEER_MOTOR = {FL_STEER_MOTOR, FR_STEER_MOTOR, RL_STEER_MOTOR, RR_STEER_MOTOR};
    static final Motor[] DRIVE_MOTOR = {FL_DRIVE_MOTOR, FR_DRIVE_MOTOR, RL_DRIVE_MOTOR, RR_DRIVE_MOTOR};

    static final Optional<Pigeon> GYRO = ofReplayable(() -> new GenericIMU("GYRO", 30));

    static {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL) {
            configureSteerConfiguration();
            configureDriveConfiguration();

            for (int i = 0; i < 4; i++) {
                configureSteerEncoder(STEER_ENCODER[i], Rotation2d.fromRotations(STEER_ENCODER_OFFSET[i]));
                configureDriveMotor(DRIVE_MOTOR[i]);
                configureSteerMotor(STEER_MOTOR[i]);
            }

            configureGyro();
        }
    }

    private static final Optional<SwerveModuleIO[]> MODULES_IO = ofReplayable(() -> new SwerveModuleIO[]{
            new RealSwerveModule(FL_DRIVE_MOTOR, FL_STEER_MOTOR, FL_STEER_ENCODER, "ModuleFL"),
            new RealSwerveModule(FR_DRIVE_MOTOR, FR_STEER_MOTOR, FR_STEER_ENCODER, "ModuleFR"),
            new RealSwerveModule(RL_DRIVE_MOTOR, RL_STEER_MOTOR, RL_STEER_ENCODER, "ModuleRL"),
            new RealSwerveModule(RR_DRIVE_MOTOR, RR_STEER_MOTOR, RR_STEER_ENCODER, "ModuleRR")
    });

    @Override
    public Optional<Pigeon> getPigeon() {
        return GYRO;
    }

    @Override
    protected Optional<SwerveModuleIO[]> getModulesIO() {
        return MODULES_IO;
    }

    private static void configureGyro() {
        GYRO.ifPresent(gyro -> {
            gyro.resetConfigurations();
            gyro.setupSignalUpdates(new PigeonSignal(PigeonSignal.SignalType.YAW, true));
        });
    }

    private static void configureSteerEncoder(Encoder steerEncoder, Rotation2d angleOffset) {
        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = CAN_CODER_INVERT;
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.ZeroToOne;
        encoderConfiguration.offsetRotations = -angleOffset.getRotations();

        steerEncoder.configure(encoderConfiguration);

        steerEncoder.setSignalUpdateFrequency(STEER_POSITION_SIGNAL);
    }

    private static void configureDriveMotor(Motor driveMotor) {
        driveMotor.setupSignalsUpdates(DRIVE_VELOCITY_SIGNAL);
        driveMotor.setupSignalsUpdates(DRIVE_POSITION_SIGNAL);

        driveMotor.setupSignalsUpdates(new MotorSignal(VOLTAGE));
        driveMotor.setupSignalsUpdates(new MotorSignal(TEMPERATURE));

        driveMotor.configure(driveMotorConfiguration);
    }

    private static void configureSteerMotor(Motor steerMotor) {
        steerMotor.setupSignalsUpdates(new MotorSignal(POSITION));

        steerMotor.configure(steerMotorConfiguration);
    }

    private static void configureDriveConfiguration() {
        driveMotorConfiguration.idleMode = DRIVE_NEUTRAL_MODE;
        driveMotorConfiguration.inverted = DRIVE_MOTOR_INVERT;

        driveMotorConfiguration.gearRatio = DRIVE_GEAR_RATIO;

        driveMotorConfiguration.statorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        driveMotorConfiguration.supplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;

        driveMotorConfiguration.slot0 = DRIVE_SLOT;

        driveMotorConfiguration.dutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        driveMotorConfiguration.dutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
    }

    private static void configureSteerConfiguration() {
        steerMotorConfiguration.slot0 = new MotorProperties.Slot(7, 0, 0, 0, 0, 0);

        steerMotorConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerMotorConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerMotorConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerMotorConfiguration.gearRatio = ANGLE_GEAR_RATIO;

        steerMotorConfiguration.closedLoopContinuousWrap = true;
    }
}
