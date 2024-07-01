package frc.robot.subsystems.flywheel.simulation;

import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class SimulatedFlywheelConstants extends FlywheelConstants {

    public static FlywheelIO[] getFlywheels() {
        return new FlywheelIO[]{
                new SimulatedFlywheel(LEFT_MOTOR, "LeftSim"),
                new SimulatedFlywheel(RIGHT_FLYWHEEL, "RightSim")
        };
    }
}
