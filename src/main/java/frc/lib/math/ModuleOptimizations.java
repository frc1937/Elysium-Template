package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleOptimizations {
    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMPS the target velocity, in meters per second
     * @param targetSteerAngle  the target steer angle
     * @return the reduced target velocity in revolutions per second
     */
    public static double reduceSkew(double targetVelocityMPS, Rotation2d targetSteerAngle, Rotation2d currentAngle) {
        final double closedLoopError = targetSteerAngle.getRadians() - currentAngle.getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));

        return targetVelocityMPS * cosineScalar;
    }
}
