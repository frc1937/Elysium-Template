package frc.lib.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
    public enum Usage {
        kAll,
        kPositionOnly,
        kVelocityOnly,
        kMinimal
    }

    /**
     * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
     * frame period of nonessential frames from 20ms to 500ms.
     *
     * <p>See
     * <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">...</a>
     * for a description of the status frames.
     *
     * @param motor The motor to adjust the status frame periods on.
     * @param usage The status frame feedback to enable. kAll is the default when a CANSparkMax is
     *     constructed.
     * @param enableFollowing Whether to enable motor following.
     */
    public static void setCANSparkFlexBusUsage(CANSparkBase motor, Usage usage, boolean enableFollowing) {
        if (enableFollowing) {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10); //follower = 0
        } else {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
        }

        //follower updateFreq = 0
        //position updateFrq= 2
        //velocity updateFreq= 1
        //voltage updateFreq= 3

        if (usage == Usage.kAll) {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);
        } else if (usage == Usage.kPositionOnly) {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20); //2 = position
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        } else if (usage == Usage.kVelocityOnly) {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20); //1 == velocity
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        } else if (usage == Usage.kMinimal) {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        }
    }


    /**
     * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
     * frame period of nonessential frames from 20ms to 500ms.
     *
     * <p>See
     * <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">...</a>
     * for a description of the status frames.
     *
     * @param motor The motor to adjust the status frame periods on.
     * @param usage The status frame feedback to enable. kAll is the default when a CANSparkMax is
     *     constructed.
     */
    public static void setCANSparkBusUsage(CANSparkBase motor, Usage usage) {
        setCANSparkFlexBusUsage(motor, usage, false);
    }
}
