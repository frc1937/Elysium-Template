package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.flywheels.real.RealFlywheelsConstants;
import frc.robot.subsystems.flywheels.simulation.SimulatedFlywheelsConstants;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public abstract class FlywheelsConstants {
    protected static final double LEFT_FLYWHEEL_DIAMETER =  Units.inchesToMeters(3);
    protected static final double RIGHT_FLYWHEEL_DIAMETER =  Units.inchesToMeters(4);

    public static final double TOLERANCE_ROTATIONS_PER_SECONDS = 0.9;

    public static final double MAXIMUM_VELOCITY_RPM = 5400;

    protected static final boolean LEFT_MOTOR_INVERT = true;
    protected static final boolean RIGHT_MOTOR_INVERT = false;

    protected static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.of(0.8).per(Second),
            Volts.of(6),
            Seconds.of(10)
    );

    static FlywheelsConstants generateConstants() {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulatedFlywheelsConstants();

        return new RealFlywheelsConstants();
    }

    protected abstract Optional<SingleFlywheelIO[]> getFlywheels();
}
