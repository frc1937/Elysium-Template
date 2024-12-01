package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonFactory;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

public class SwerveConstants {
    public static final double DRIVE_GEAR_RATIO = (6.75);
    public static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    public static final double WHEEL_DIAMETER = 0.0465117190934167*2;
//            Meters.convertFrom(4, Inch);

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final double MAX_SPEED_MPS = 5.1;
    public static final double MAX_ROTATION_RAD_PER_S = 3 * Math.PI;

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();
    
    public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    };

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.15,
            ROTATION_NEUTRAL_DEADBAND = 0.15;

    /**
     * Units of RADIANS for everything.
     */
    static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
//            1.9, 0, 0.0011,
            3.9, 0, 0.05,
            new TrapezoidProfile.Constraints(360, 360)
    );

    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(1, 0, 0),
            MAX_SPEED_MPS,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig(true, false)
    );

    static final Pigeon GYRO = PigeonFactory.createIMU("GYRO", 30);

    static {
        configureGyro();
        configureRotationController();
    }

    private static void configureGyro() {
        GYRO.configurePigeon(new PigeonConfiguration());
        GYRO.setupSignalUpdates(PigeonSignal.YAW, true);
    }

    private static void configureRotationController() {
        ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        ROTATION_CONTROLLER.setTolerance(1.5);
    }
}
