package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.Properties;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

import java.util.Optional;

import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;

public class RealSwerveConstants extends SwerveConstants {
    private static final Optional<SwerveModuleIO[]> MODULES_IO = ofReplayable(() -> new SwerveModuleIO[]{
            new RealSwerveModuleIO(RealSwerveModuleConstants.FRONT_LEFT_SWERVE_MODULE_CONSTANTS, "FrontLeft"),
            new RealSwerveModuleIO(RealSwerveModuleConstants.FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, "FrontRight"),
            new RealSwerveModuleIO(RealSwerveModuleConstants.REAR_LEFT_SWERVE_MODULE_CONSTANTS, "RearLeft"),
            new RealSwerveModuleIO(RealSwerveModuleConstants.REAR_RIGHT_SWERVE_MODULE_CONSTANTS, "RearRight")
    });

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(6.5, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);


    private static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );

    static final Optional<WPI_PigeonIMU> GYRO = ofReplayable(() -> new WPI_PigeonIMU(30));

    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, false);

    private static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS,
            MAX_SPEED_MPS,
            DRIVE_BASE_RADIUS,
            REPLANNING_CONFIG
    );

    static {
        if (GlobalConstants.CURRENT_MODE != GlobalConstants.Mode.REPLAY)
            configureGyro();
    }

    private static void configureGyro() {
        GYRO.get().configFactoryDefault();
    }

    @Override
    public Optional<WPI_PigeonIMU> getPigeon() {
        return GYRO;
    }

    @Override
    protected Optional<SwerveModuleIO[]> getModulesIO() {
        return MODULES_IO;
    }

    @Override
    protected HolonomicPathFollowerConfig getPathFollowerConfig() {
        return HOLONOMIC_PATH_FOLLOWER_CONFIG;
    }

    @Override
    protected PIDController getTranslationsController() {
        return TRANSLATION_PID_CONTROLLER;
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

        driveMotor.setSignalUpdateFrequency(Properties.SignalType.VOLTAGE, 50);
        driveMotor.setSignalUpdateFrequency(Properties.SignalType.TEMPERATURE, 50);

        driveMotor.configure(driveMotorConfiguration);
    }

    private static void configureSteerMotor(Motor steerMotor) {
        steerMotor.setSignalUpdateFrequency(Properties.SignalType.POSITION, ODOMETRY_FREQUENCY_HERTZ);
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
        steerMotorConfiguration.slot0 = new MotorProperties.Slot(5, 0, 0, 0, 0, 0);

        steerMotorConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerMotorConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerMotorConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerMotorConfiguration.conversionFactor = ANGLE_GEAR_RATIO;

        steerMotorConfiguration.ClosedLoopContinousWrap = true;
    }
}
