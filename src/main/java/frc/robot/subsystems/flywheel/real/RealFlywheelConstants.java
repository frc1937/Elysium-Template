package frc.robot.subsystems.flywheel.real;

import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class RealFlywheelConstants extends FlywheelConstants {

    private static final Motor LEFT_FLYWHEEL_MOTOR = new GenericSpark();
    private static final Motor RIGHT_FLYWHEEL_MOTOR = new GenericSpark();
    //TODO: add all constants and make this work.

    private static final MotorProperties.Slot LEFT_SLOT = new MotorProperties.Slot(

    );

    private static final MotorProperties.Slot RIGHT_SLOT = new MotorProperties.Slot(
    )


    public static FlywheelIO[] getFlywheels() {
        return new RealFlywheel[]{
                new RealFlywheel(LEFT_FLYWHEEL_MOTOR, "LeftSim"),
                new RealFlywheel(RIGHT_FLYWHEEL_MOTOR, "RightSim")
        };
    }
}
