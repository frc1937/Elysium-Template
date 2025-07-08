package frc.lib.generic;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;

public class PID {
    private final double kS;
    private double kP;
    private double kI;
    private double kD;
    private double iZone = Double.POSITIVE_INFINITY;

    private double maximumIntegral = 1.0;
    private double minimumIntegral = -1.0;

    private double maximumInput;
    private double minimumInput;

    private boolean continuous;

    private double positionError;
    private double velocityError;
    private double prevError;
    private double totalError;

    private double m_positionTolerance = 0.05;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;

    private double m_setpoint;
    private double m_measurement;

    private boolean m_haveMeasurement;
    private boolean m_haveSetpoint;

    public PID(double kp, double ki, double kd) {
        this(kp, ki, kd, 0.0);
    }

    public PID(PIDConstants constants) {
        this(constants.kP, constants.kI, constants.kD, 0);
    }

    public PID(double kp, double ki, double kd, double kS) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.kS = kS;

        if (kp < 0.0) {
            throw new IllegalArgumentException("Kp must be a non-negative number!");
        }
        if (ki < 0.0) {
            throw new IllegalArgumentException("Ki must be a non-negative number!");
        }
        if (kd < 0.0) {
            throw new IllegalArgumentException("Kd must be a non-negative number!");
        }
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return kP;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double kp) {
        kP = kp;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return kI;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double ki) {
        kI = ki;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return kD;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double kd) {
        kD = kd;
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return iZone;
    }

    /**
     * Sets the IZone range. When the absolute value of the position error is greater than IZone, the
     * total accumulated error will reset to zero, disabling integral gain until the absolute value of
     * the position error is less than IZone. This is used to prevent integral windup. Must be
     * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
     * of {@link Double#POSITIVE_INFINITY} disables IZone functionality.
     *
     * @param iZone Maximum magnitude of error to allow integral control.
     * @throws IllegalArgumentException if iZone &lt; 0
     */
    public void setIZone(double iZone) {
        if (iZone < 0) {
            throw new IllegalArgumentException("IZone must be a non-negative number!");
        }
        this.iZone = iZone;
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return m_positionTolerance;
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return m_velocityTolerance;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return m_setpoint;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;

        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            positionError = m_setpoint - m_measurement;
        }

        velocityError = (positionError - prevError) / 0.02;
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return m_haveMeasurement
                && m_haveSetpoint
                && Math.abs(positionError) < m_positionTolerance
                && Math.abs(velocityError) < m_velocityTolerance;
    }

    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        continuous = false;
    }

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    public boolean isContinuousInputEnabled() {
        return continuous;
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>When the cap is reached, the integrator value is added to the controller output rather than
     * the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        this.minimumIntegral = minimumIntegral;
        this.maximumIntegral = maximumIntegral;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return positionError;
    }

    /**
     * Returns the velocity error.
     *
     * @return The velocity error.
     */
    public double getVelocityError() {
        return velocityError;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        m_measurement = measurement;
        prevError = positionError;
        m_haveMeasurement = true;

        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            positionError = m_setpoint - m_measurement;
        }

        velocityError = (positionError - prevError) / 0.02;

        if (Math.abs(positionError) > iZone) {
            totalError = 0;
        } else if (kI != 0) {
            totalError =
                    MathUtil.clamp(
                            totalError + positionError * 0.02,
                            minimumIntegral / kI,
                            maximumIntegral / kI);
        }

        final double output = kP * positionError + kI * totalError + kD * velocityError;
        return output + getNominalOutput(output);
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        positionError = 0;
        prevError = 0;
        totalError = 0;
        velocityError = 0;
        m_haveMeasurement = false;
    }

    private double getNominalOutput(double output) {
        if (output < 0 && output > -kS) {
            return -kS;
        } else if (output > 0 && output < kS) {
            return kS;
        }

        return 0;
    }
}
