package frc.lib.math;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Conversions {
    public static final double
            DEGREES_PER_ROTATIONS = 360,
            HUNDRED_MS_PER_SEC = 10,
            SEC_PER_MIN = 60;

    /**
     * Converts degrees to rotations.
     *
     * @param degrees degrees
     * @return rotations
     */
    public static double degreesToRotations(double degrees) {
        return degrees / DEGREES_PER_ROTATIONS;
    }

    public static double rpmToRotationsPerSecond(double rpm) {
        return rpm / (SEC_PER_MIN);
    }

    /**
     * Converts rotations to degrees.
     *
     * @param rotations rotations
     * @return degrees
     */
    public static double rotationsToDegrees(double rotations) {
        return rotations * DEGREES_PER_ROTATIONS;
    }

    /**
     * Returns tangential velocity in metres per second.
     *
     * @param rpm           - rotations per minute
     * @param wheelDiameter - wheel diameter in metres
     * @return the tangential velocity in metres per second
     */
    public static double tangentialVelocityFromRPM(double rpm, double wheelDiameter) {
        return rotationsPerSecondToMetersPerSecond(rpm / SEC_PER_MIN, wheelDiameter);
    }

    /**
     * Returns the rotations per minute
     *
     * @param tangentialVelocityMPS - metres per second
     * @param wheelDiameterMeters         - wheel diameter in meters
     * @return rotations per minute
     */
    public static double rpmFromTangentialVelocity(double tangentialVelocityMPS, double wheelDiameterMeters) {
        return SEC_PER_MIN * metersPerSecondToRotationsPerSecond(tangentialVelocityMPS, wheelDiameterMeters);
    }

    /**
     * Converts rotations to meters.
     *
     * @param rotations rotations
     * @return meters
     */
    public static double rotationsToMeters(double rotations, double wheelDiameter) {
        return rotations * wheelDiameter * Math.PI;
    }

    /**
     * Converts rotations per second to meters per second.
     * This is the same as converting rotations to metres.
     *
     * @param rotationsPerSecond rotations per second
     * @return meters per second
     */
    public static double rotationsPerSecondToMetersPerSecond(double rotationsPerSecond, double wheelDiameter) {
        return rotationsToMeters(rotationsPerSecond, wheelDiameter);
    }

    /**
     * Converts metres per second to rotations per second.
     * This is the same as converting metres to rotations.
     *
     * @param velocityMetersPerSecond      the velocity in metres per second
     * @param wheelDiameterMeters the wheel diameter in metres
     * @return the rotations
     */
    public static double metersPerSecondToRotationsPerSecond(double velocityMetersPerSecond, double wheelDiameterMeters) {
        return metresToRotations(velocityMetersPerSecond, wheelDiameterMeters);
    }

    /**
     * Converts metres to rotations.
     * This is the same as converting metres per second to rotations per second.
     *
     * @param metres      the distance
     * @param wheelDiameter the wheel diameter
     * @return the rotations
     */
    public static double metresToRotations(double metres, double wheelDiameter) {
        return metres / (wheelDiameter * Math.PI);
    }
    

    /**
     * Converts a target output percentage output to voltage when voltage compensation is enabled.
     * The voltage compensation saturation determines what voltage represents 100% output.
     * The compensated power is the voltage represented by a percentage of the saturation voltage.
     *
     * @param power      the target percentage output
     * @param saturation the configured saturation which represents 100% output
     * @return the percentage output to achieve the target voltage
     */
    public static double compensatedPowerToVoltage(double power, double saturation) {
        return power * saturation;
    }

    public static double hertzToMs(double hertz) {
        return 1000 / hertz;
    }

    /**
     * Scales a TrapezoidProfile.Constraints object by a given percentage.
     *
     * @param constraints the constraints to scale
     * @param percentage  the percentage of speed
     * @return the scaled constraints
     */
    public static TrapezoidProfile.Constraints scaleConstraints(TrapezoidProfile.Constraints constraints, double percentage) {
        return new TrapezoidProfile.Constraints(constraints.maxVelocity * (percentage / 100), constraints.maxAcceleration * (percentage / 100));
    }
}
