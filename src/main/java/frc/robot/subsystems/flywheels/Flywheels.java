package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.commands.ExecuteEndCommand;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.flywheels;

public class Flywheels extends SubsystemBase {
    public Command setTargetVelocity(double velocityRPS) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTargetVelocity(velocityRPS),
                this::stop,
                this
        );
    }

    public Command setTargetTangentialVelocity(double velocityMPS) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTangentialVelocity(velocityMPS),
                this::stop,
                this
        );
    }

    @Override
    public void periodic() {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.periodic();
        }
    }

    public boolean hasReachedTarget() {
        for (SingleFlywheel flywheel : flywheels) {
            if (!flywheel.hasReachedTarget()) {
                return false;
            }
        }
        return true;
    }

    public void stop() {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.stop();
        }
    }

    public void setFlywheelsTangentialVelocity(double targetMPS) {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.setTargetTangentialVelocity(targetMPS);
        }
    }

    public void setFlywheelsTargetVelocity(double targetRPS) {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.setTargetVelocity(targetRPS);
        }
    }
}
