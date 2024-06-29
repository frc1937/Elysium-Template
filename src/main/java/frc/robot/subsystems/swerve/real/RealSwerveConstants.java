package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.Properties;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.GenericCanCoder;
import frc.lib.generic.motor.*;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;

public class RealSwerveConstants extends SwerveConstants {
    static final WPI_PigeonIMU GYRO = new WPI_PigeonIMU(30);

    static final MotorProperties.IdleMode ANGLE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;
    static final MotorProperties.IdleMode DRIVE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final double OPEN_LOOP_RAMP = 0.1;
    static final double CLOSED_LOOP_RAMP = 0.0;

    static final boolean CAN_CODER_INVERT = false;
    static final boolean ANGLE_MOTOR_INVERT = true;
    static final boolean DRIVE_MOTOR_INVERT = false;

    static final double MAX_SPEED_MPS = 5.1;

    static final int ANGLE_CURRENT_LIMIT = 30;
    static final int DRIVE_SUPPLY_CURRENT_LIMIT = 35;
    static final int DRIVE_STATOR_CURRENT_LIMIT = 50;

    static final double DRIVE_GEAR_RATIO = (6.75);
    static final double ANGLE_GEAR_RATIO = (7.0 / 150.0);

    static final MotorProperties.Slot DRIVE_SLOT = new MotorProperties.Slot(0.053067, 0.0, 0.0,
            0.10861,
            0.023132,
            0.27053);

    public static final Motor FL_STEER_MOTOR = new GenericSpark(11, MotorProperties.SparkType.MAX),
            FR_STEER_MOTOR = new GenericSpark(10, MotorProperties.SparkType.MAX),
            RL_STEER_MOTOR = new GenericSpark(6, MotorProperties.SparkType.MAX),
            RR_STEER_MOTOR = new GenericSpark(9, MotorProperties.SparkType.MAX);

    public static final Motor FL_DRIVE_MOTOR = new GenericTalonFX(14),
            FR_DRIVE_MOTOR = new GenericTalonFX(3),
            RL_DRIVE_MOTOR = new GenericTalonFX(13),
            RR_DRIVE_MOTOR = new GenericTalonFX(2);

    public static final Encoder FL_STEER_ENCODER = new GenericCanCoder(18),
            FR_STEER_ENCODER = new GenericCanCoder(20),
            RL_STEER_ENCODER = new GenericCanCoder(19),
            RR_STEER_ENCODER = new GenericCanCoder(21);

    static final double[] STEER_ENCODER_OFFSET = {0.677246, 0.282715, 0.533447, 0.313721};

    static final Encoder[] STEER_ENCODER = {FL_STEER_ENCODER, FR_STEER_ENCODER, RL_STEER_ENCODER, RR_STEER_ENCODER};
    static final Motor[] STEER_MOTOR = {FL_STEER_MOTOR, FR_STEER_MOTOR, RL_STEER_MOTOR, RR_STEER_MOTOR};
    static final Motor[] DRIVE_MOTOR = {FL_DRIVE_MOTOR, FR_DRIVE_MOTOR, RL_DRIVE_MOTOR, RR_DRIVE_MOTOR};

    static final MotorConfiguration steerMotorConfiguration = new MotorConfiguration();
    static final MotorConfiguration driveMotorConfiguration = new MotorConfiguration();

    static {
        configureSteerConfiguration();
        configureDriveConfiguration();

        for (int i = 0; i < 4; i++) {
            configureSteerEncoder(STEER_ENCODER[i], Rotation2d.fromRotations(STEER_ENCODER_OFFSET[i]));
            configureDriveMotor(DRIVE_MOTOR[i]);
            configureSteerMotor(STEER_MOTOR[i]);
        }

        GYRO.configFactoryDefault();
    }

    @Override
    protected SwerveModuleIO[] getSwerveModules() {
        return new SwerveModuleIO[]{
                new RealSwerveModule(FL_DRIVE_MOTOR, FL_STEER_MOTOR, FL_STEER_ENCODER, "ModuleFL"),
                new RealSwerveModule(FR_DRIVE_MOTOR, FR_STEER_MOTOR, FR_STEER_ENCODER, "ModuleFR"),
                new RealSwerveModule(RL_DRIVE_MOTOR, RL_STEER_MOTOR, RL_STEER_ENCODER, "ModuleRL"),
                new RealSwerveModule(RR_DRIVE_MOTOR, RR_STEER_MOTOR, RR_STEER_ENCODER, "ModuleRR")
        };
    }

    private static void configureSteerEncoder(Encoder steerEncoder, Rotation2d angleOffset) {
        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = CAN_CODER_INVERT;
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.ZeroToOne;
        encoderConfiguration.offsetRotations = -angleOffset.getRotations();

        steerEncoder.configure(encoderConfiguration);

        steerEncoder.setSignalUpdateFrequency(Properties.SignalType.POSITION, ODOMETRY_FREQUENCY_HERTZ);
    }

    private static void configureDriveMotor(Motor driveMotor) {
        driveMotor.setSignalUpdateFrequency(Properties.SignalType.VELOCITY, ODOMETRY_FREQUENCY_HERTZ);
        driveMotor.setSignalUpdateFrequency(Properties.SignalType.POSITION, ODOMETRY_FREQUENCY_HERTZ);

        driveMotor.setSignalUpdateFrequency(Properties.SignalType.TEMPERATURE, 50);
        driveMotor.configure(driveMotorConfiguration);
    }

    private static void configureSteerMotor(Motor steerMotor) {
        steerMotor.setSignalUpdateFrequency(Properties.SignalType.POSITION, 50);

        steerMotor.configure(steerMotorConfiguration);
    }

    private static void configureDriveConfiguration() {
        driveMotorConfiguration.idleMode = DRIVE_NEUTRAL_MODE;
        driveMotorConfiguration.inverted = DRIVE_MOTOR_INVERT;

        driveMotorConfiguration.conversionFactor = DRIVE_GEAR_RATIO;

        driveMotorConfiguration.statorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        driveMotorConfiguration.supplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;

        driveMotorConfiguration.slot0 = DRIVE_SLOT;

        driveMotorConfiguration.dutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        driveMotorConfiguration.dutyCycleCloseLoopRampPeriod = CLOSED_LOOP_RAMP;
    }

    private static void configureSteerConfiguration() {
        steerMotorConfiguration.slot0 = new MotorProperties.Slot(1, 0, 0, 1, 0, 0);

        steerMotorConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerMotorConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerMotorConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerMotorConfiguration.conversionFactor = ANGLE_GEAR_RATIO;
    }
}
