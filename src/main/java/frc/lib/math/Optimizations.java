package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveConstants;

public class Optimizations {
    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMPS the target velocity, in meters per second
     * @param targetSteerAngle  the target steer angle
     * @return the reduced target velocity in metres per second
     */
    public static double reduceSkew(double targetVelocityMPS, Rotation2d targetSteerAngle, Rotation2d currentAngle) {
        final double closedLoopError = targetSteerAngle.getRadians() - currentAngle.getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));

        return targetVelocityMPS * cosineScalar;
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds The chassis speeds to fix skewing for
     * @param lastTimestamp The timestamp of the last loop
     * @return the fixed speeds
     */
    public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds, double lastTimestamp) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double difference = currentTimestamp - lastTimestamp;

        return ChassisSpeeds.discretize(chassisSpeeds, difference);
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    public static boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }
}
