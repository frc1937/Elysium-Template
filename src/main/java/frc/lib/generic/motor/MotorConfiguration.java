package frc.lib.generic.motor;

/**
 * Generic configurations class with default values
 */
public class MotorConfiguration {
    public boolean inverted = false;
    public MotorProperties.IdleMode idleMode = MotorProperties.IdleMode.BRAKE;

    public MotorProperties.Slot slot0 = new MotorProperties.Slot(0, 0, 0, 0, 0, 0, 0, null);
    public MotorProperties.Slot slot1 = new MotorProperties.Slot(0, 0, 0, 0, 0, 0, 0, null);
    public MotorProperties.Slot slot2 = new MotorProperties.Slot(0, 0, 0, 0, 0, 0, 0, null);

    public int slotToUse = 0;

    /**
     * If non-zero, this determines how much time to ramp from 0% output to 100% during open-loop modes.
     * @| Minimum Value: 0
     * @| Maximum Value: 1
     * @| Default Value: 0
     * @| Units: sec
     */
    public double dutyCycleOpenLoopRampPeriod = 0;
    /**
     * If non-zero, this determines how much time to ramp from 0% output to 100% during closed-loop modes.
     * @| Minimum Value: 0
     * @| Maximum Value: 1
     * @| Default Value: 0
     * @| Units: sec
     */
    public double dutyCycleCloseLoopRampPeriod = 0;

    public double supplyCurrentLimit = -1, statorCurrentLimit = -1;

    //Todo: Check if this is 1 / gear OR gear.
    public double conversionFactor = 1;
}
