package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class FlywheelConstants {
    protected static final double LEFT_FLYWHEEL_DIAMETER =  Units.inchesToMeters(3);
    protected static final double RIGHT_FLYWHEEL_DIAMETER =  Units.inchesToMeters(4);

    public static final double TOLERANCE_ROTATIONS_PER_SECONDS = 0.9;

    public static final double MAXIMUM_VELOCITY_RPM = 5400;

    protected static final boolean LEFT_MOTOR_INVERT = false;
    protected static final boolean RIGHT_MOTOR_INVERT = true;

    protected static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.of(0.8).per(Second),
            Volts.of(6),
            Seconds.of(10)
    );

    protected static FlywheelIO[] getFlywheels() {
        return new FlywheelIO[]{
                new FlywheelIO("Left"),
                new FlywheelIO("Right")
        };
    }
}
