package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.real.RealSwerveConstants;
import frc.robot.subsystems.swerve.simulationswerve.SimulationSwerveConstants;

import java.util.Optional;
import java.util.function.Supplier;

public abstract class SwerveConstants {
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.05,
            ROTATION_TOLERANCE_DEGREES = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.3;

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),

            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    static final LoggedTunableNumber ROTATION_KP = new LoggedTunableNumber("Swerve/RotationKP", 1);

    static final LoggedTunableNumber ROTATION_MAX_VELOCITY = new LoggedTunableNumber("Swerve/RotationMaxVelocity", Math.PI),
            ROTATION_MAX_ACCELERATION = new LoggedTunableNumber("Swerve/RotationMaxAcceleration", Math.PI );

    /** Units of RADIANS for everything. */
    public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            ROTATION_KP.get(), 0, 0.015,
            new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY.get(), ROTATION_MAX_ACCELERATION.get())
    );

    protected static <T> Optional<T> ofReplayable(Supplier<T> value) {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return Optional.empty();
        return Optional.of(value.get());
    }

    static SwerveConstants generateConstants() {
        ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        ROTATION_CONTROLLER.setTolerance(Units.degreesToRadians(0.5));

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new RealSwerveConstants();

        return new SimulationSwerveConstants();
    }

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();

    protected abstract Optional<WPI_PigeonIMU> getPigeon();

    protected abstract PIDController getTranslationsController();

    protected abstract Optional<SwerveModuleIO[]> getModulesIO();

    protected abstract HolonomicPathFollowerConfig getPathFollowerConfig();

    protected abstract double getMaxSpeedMetersPerSecond();

    protected abstract double getMaxRotationalSpeedRadiansPerSecond();
}
