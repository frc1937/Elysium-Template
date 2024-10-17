package frc.lib.generic.hardware.motor.hardware.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

public class SparkCommon {
    private SparkCommon() {
    }

    public enum MotionType {
        POSITION_S_CURVE, POSITION_TRAPEZOIDAL, VELOCITY_SIMPLE, VELOCITY_TRAPEZOIDAL, POSITION_SIMPLE
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    protected static void optimizeBusUsage(CANSparkBase spark) {
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, 32767);
    }

    protected static Feedforward.Type configureFeedforward(MotorProperties.Slot slot) {
        Feedforward.Type feedforwardType = Feedforward.Type.SIMPLE;

        if (slot.gravityType() == MotorProperties.GravityType.ARM) {
            feedforwardType = Feedforward.Type.ARM;
        }

        if (slot.gravityType() == MotorProperties.GravityType.ELEVATOR) {
            feedforwardType = Feedforward.Type.ELEVATOR;
        }

        feedforwardType.setFeedforwardConstants(
                slot.kS(),
                slot.kV(),
                slot.kA(),
                slot.kG()
        );

        return feedforwardType;
    }

    protected static boolean hasNoNewGoal(TrapezoidProfile.State newGoal, TrapezoidProfile.State goalState, boolean hasStoppedOccurred, double lastProfileCalculationTimestamp) {
        return goalState != null
                && goalState.equals(newGoal)
                && !hasStoppedOccurred
                && (Logger.getRealTimestamp() - lastProfileCalculationTimestamp <= 100000); //(0.1 sec has passed)
    }
}
