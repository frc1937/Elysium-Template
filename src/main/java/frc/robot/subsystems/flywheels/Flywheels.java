package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.commands.ExecuteEndCommand;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
    private final FlywheelsInputsAutoLogged flywheelsInputs = new FlywheelsInputsAutoLogged();
    private final FlywheelsIO flywheels = FlywheelsIO.generateIO();

    private final FlywheelsConstants constants = FlywheelsConstants.generateConstants();
    private final SingleFlywheelIO[] singleFlywheels = getFlywheels();

    public Command setFlywheelsTargetVelocity(double targetRPS) {
        return new ExecuteEndCommand(
                () -> setTargetVelocity(targetRPS),
                this::stop,
                this
        );
    }

    public Command setFlywheelsTangentialVelocity(double velocityMetersPerSecond) {
        return new ExecuteEndCommand(
                () -> setTargetTangentialVelocity(velocityMetersPerSecond),
                this::stop,
                this
        );
    }

    @Override
    public void periodic() {
        flywheels.refreshInputs(flywheelsInputs);
        Logger.processInputs("Flywheels", flywheelsInputs);

        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.periodic();
    }

    public boolean hasReachedTarget() {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels) {
            if(!currentFlywheel.hasReachedTarget())
                return false;
        }

        return true;
    }

    private void stop() {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.stop();
    }

    private void setTargetVelocity(double targetRPS) {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.setTargetVelocityRPS(targetRPS);
    }

    private void setTargetTangentialVelocity(double velocityMetersPerSecond) {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.setTargetTangentialVelocity(velocityMetersPerSecond);
    }

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
