package frc.robot.subsystems.flywheel;

public class FlywheelConstants {

    public static FlywheelIO[] getFlywheels() {
        return new FlywheelIO[]{
                new FlywheelIO("Left"),
                new FlywheelIO("Right")
        };
    }
}
