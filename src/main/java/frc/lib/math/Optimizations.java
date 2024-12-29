package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Optimizations {
    /**
     * Determines if the robot is currently experiencing a collision based on accelerometer data.
     * This method calculates the resultant acceleration from the X and Y axes of the accelerometer,
     * then multiplies it by the gravitational constant to convert it into a force value.
     * If this force exceeds a predefined minimum threshold, it is considered that a collision has occurred.
     *
     * @return true if the calculated force indicates a collision; false otherwise.
     */
    public static boolean isColliding() {
//        final float xAcceleration = (float) ROBORIO_ACCELEROMETER.getX();
//        final float yAcceleration = (float) ROBORIO_ACCELEROMETER.getY();
//
//        return Math.hypot(xAcceleration, yAcceleration) * GRAVITY > MINIMUM_ACCELERATION_FOR_COLLISION;
        return false; //TODO: USE GYRO INSTEAD! THE ACCELEROMETER IS DEEPLY FLAWED!
    }

    /**
     * Gets the skidding ratio from the latest module state, that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the ratio between the maximum and minimum magnitude of the "translational" part of the speed of the modules
     *
     * @param swerveDriveKinematics the kinematics
     * @param swerveStatesMeasured  the swerve states measured from the modules
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     */
    public static double getSkiddingRatio(final SwerveDriveKinematics swerveDriveKinematics, final SwerveModuleState[] swerveStatesMeasured) {
        final double angularVelocity = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] rotationalStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocity));

        final double[] translationalSpeeds = new double[swerveStatesMeasured.length];

        for (int i = 0; i < swerveStatesMeasured.length; i++) {
            final Translation2d measuredVelocity = convertToVelocityVector(swerveStatesMeasured[i]),
                    rotationalVelocity = convertToVelocityVector(rotationalStates[i]),
                    translationalVelocity = measuredVelocity.minus(rotationalVelocity);

            translationalSpeeds[i] = translationalVelocity.getNorm();
        }

        final double maxSpeed = Arrays.stream(translationalSpeeds).max().orElse(0);
        final double minSpeed = Arrays.stream(translationalSpeeds).min().orElse(Double.POSITIVE_INFINITY);

        return maxSpeed / minSpeed;
    }

    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMPS the target velocity, in meters per second
     * @param targetSteerAngle  the target steer angle
     * @return the reduced target velocity in metres per second
     */
    public static double reduceSkew(double targetVelocityMPS, Rotation2d targetSteerAngle, Rotation2d currentAngle) {
        final double closedLoopError = targetSteerAngle.minus(currentAngle).getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));

        return targetVelocityMPS * cosineScalar;
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePositionRevolutions the position in revolutions
     * @param moduleAngle              the angle of the module
     * @return the distance without the coupling
     */
    public static double removeCouplingFromRevolutions(double drivePositionRevolutions, Rotation2d moduleAngle, double couplingRatio) {
        final double coupledAngle = moduleAngle.getRotations() * couplingRatio;
        return drivePositionRevolutions - coupledAngle;
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
        final double currentTimestamp = Logger.getTimestamp();
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

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }


    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * Converts a {@link SwerveModuleState} to a {@link Translation2d} vector representing the velocity.
     * This method takes the speed and angle from the swerve module state and creates a 2D translation vector.
     * The speed is used as the magnitude of the vector, and the angle is used to determine the direction.
     *
     * @param state The {@link SwerveModuleState} to convert.
     * @return A {@link Translation2d} representing the velocity vector of the swerve module.
     */
    private static Translation2d convertToVelocityVector(final SwerveModuleState state) {
        return new Translation2d(state.speedMetersPerSecond, state.angle);
    }
}
