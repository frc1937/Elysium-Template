package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public abstract class SwerveConstants {
    public static final double DRIVE_GEAR_RATIO = (6.75);
    public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0);

    public static final double WHEEL_DIAMETER = Meters.convertFrom(4, Inch);

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final double MAX_SPEED_MPS = 5.1;
    public static final double MAX_ROTATION_RAD_PER_S = 3 * Math.PI;

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),

            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    static final LoggedTunableNumber
            ROTATION_KP = new LoggedTunableNumber("Swerve/RotationKP", 1),
            ROTATION_MAX_VELOCITY = new LoggedTunableNumber("Swerve/RotationMaxVelocity", Math.PI),
            ROTATION_MAX_ACCELERATION = new LoggedTunableNumber("Swerve/RotationMaxAcceleration", Math.PI / 2);

    /**
     * Units of RADIANS for everything.
     */
    public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            ROTATION_KP.get(), 0, 0.015,
            new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY.get(), ROTATION_MAX_ACCELERATION.get())
    );

    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(1, 0, 0),
            new PIDConstants(2, 0, 0),
            MAX_SPEED_MPS,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig(true, false)
    );

    protected static <T> Optional<T> ofReplayable(Supplier<T> value) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return Optional.empty();

        return Optional.of(value.get());
    }

    static SwerveConstants generateConstants() {
        ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        ROTATION_CONTROLLER.setTolerance(Units.degreesToRadians(0.5));

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulationSwerveConstants();

        return new RealSwerveConstants();
    }

    protected abstract Optional<WPI_PigeonIMU> getPigeon();
    protected abstract Optional<SwerveModuleIO[]> getModulesIO();
}
