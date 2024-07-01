package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO[] flywheels = FlywheelIO.generateFlywheels();

    public Command setFlywheelTarget(double velocityRotationsPerSecond) {
        return new FunctionalCommand(
                () -> setFlywheelTargetVelocity(velocityRotationsPerSecond),
                () -> {},
                (interrupt) -> stop(),
                () -> false,
                this
        );
    }

    public boolean hasReachedTarget() {
        for (FlywheelIO flywheel : flywheels) {
            if (!flywheel.hasReachedTarget()) {
                return false;
            }
        }

        return true;
    }

    private void setFlywheelTargetVelocity(double velocityRotationsPerSecond) {
        for (FlywheelIO flywheel : flywheels) {
            flywheel.setTargetVelocity(velocityRotationsPerSecond);
        }
    }

    private void stop() {
        for (FlywheelIO flywheel : flywheels) {
            flywheel.stop();
            flywheel.setTargetVelocity(0);
        }
    }

    @Override
    public void periodic() {

        for (FlywheelIO flywheel : flywheels) {
            flywheel.periodic();
        }
    }
}
