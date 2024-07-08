package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

public class Flywheels extends SubsystemBase {
    private final FlywheelsInputsAutoLogged flywheelsInputs = new FlywheelsInputsAutoLogged();
    private final FlywheelsIO flywheels = FlywheelsIO.generateIO();

    private final FlywheelsConstants constants = FlywheelsConstants.generateConstants();
    private final SingleFlywheelIO[] flywheelsIO = getFlywheels();

    private SingleFlywheelIO[] getFlywheels() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new SingleFlywheelIO[]{
                    new SingleFlywheelIO("Left"),
                    new SingleFlywheelIO("Right")
            };
        }

        return constants.getFlywheels().get();
    }
}
