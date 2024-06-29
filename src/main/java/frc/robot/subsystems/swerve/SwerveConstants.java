package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.real.RealSwerveConstants;
import frc.robot.subsystems.swerve.simulation.SimulatedSwerveConstants;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

public abstract class SwerveConstants {
    public static final double WHEEL_DIAMETER = Meters.convertFrom(4, Inch);

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();


    static SwerveConstants generateConstants() {
        if(GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedSwerveConstants();
        }

        return new RealSwerveConstants();
    }


    protected abstract SwerveModuleIO[] getSwerveModules();
}
