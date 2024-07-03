package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),

            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    protected static <T> Optional<T> ofReplayable(Supplier<T> value) {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return Optional.empty();
        return Optional.of(value.get());
    }

    static SwerveConstants generateConstants() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new RealSwerveConstants();

        return new SimulationSwerveConstants();
    }

    public abstract double getDriveRadiusMeters();

    protected abstract Optional<Pigeon2> getPigeon();

    protected abstract ProfiledPIDController getProfiledRotationController();

    protected abstract PIDController getTranslationsController();

    protected abstract Optional<SwerveModuleIO[]> getModulesIO();

    protected abstract HolonomicPathFollowerConfig getPathFollowerConfig();

    protected abstract double getMaxSpeedMetersPerSecond();

    protected abstract double getMaxRotationalSpeedRadiansPerSecond();
}
