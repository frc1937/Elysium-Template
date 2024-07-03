package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class ArmConstants {
    protected static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(2),
            Seconds.of(10)
    );

    public static final double TOLERANCE_ROTATIONS = Units.degreesToRotations(0.5);
}
