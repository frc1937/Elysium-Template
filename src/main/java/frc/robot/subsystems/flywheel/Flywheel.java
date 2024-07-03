package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.flywheel.real.RealFlywheelConstants;
import frc.robot.subsystems.flywheel.simulation.SimulatedFlywheelConstants;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO[] flywheels = generateFlywheels();

    public Command setFlywheelTarget(double velocityRotationsPerSecond) {
        return Commands.startEnd(
                () -> setFlywheelTargetVelocity(velocityRotationsPerSecond),
                this::stop,
                this
        );
    }

    public Command setFlywheelTargetTangentialVelocity(double velocityMetersPerSecond) {
        return Commands.startEnd(
                () -> setFlywheelTangentialVelocity(velocityMetersPerSecond),
                this::stop,
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

    private void setFlywheelTangentialVelocity(double velocityMetersPerSecond) {
        for (FlywheelIO flywheel : flywheels) {
            flywheel.setTargetVelocity(Conversions.mpsToRps(velocityMetersPerSecond, flywheel.getFlywheelDiameter()));
        }
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

    private FlywheelIO[] generateFlywheels() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return RealFlywheelConstants.getFlywheels();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return SimulatedFlywheelConstants.getFlywheels();

        return FlywheelConstants.getFlywheels();
    }
}
