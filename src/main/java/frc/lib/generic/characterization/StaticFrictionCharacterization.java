package frc.lib.generic.characterization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;

public class StaticFrictionCharacterization extends Command {
    private final Motor motor;
    private final MotorProperties.IdleMode startingIdleMode;

    private int movedCounter = 0;
    private double rampVoltage;

    public StaticFrictionCharacterization(GenericSubsystem requirement, Motor motor, double startingRampVoltageValue) {
        this.rampVoltage = startingRampVoltageValue;
        this.motor = motor;

        startingIdleMode = motor.getCurrentConfiguration().idleMode;
        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        motor.stopMotor();
        motor.setIdleMode(MotorProperties.IdleMode.BRAKE);
    }

    @Override
    public void execute() {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, rampVoltage);

        if (motor.getSystemVelocity() != 0) {
            System.out.print(
                    "\n<~~~~~~~~~~~~~~>" +
                    "\nMECHANISM " + motor.getName() + " MOVED AT " + rampVoltage + " with velocity of " + motor.getSystemVelocity() +
                    "\n<~~~~~~~~~~~~~~>");

            movedCounter++;
        } else {
            rampVoltage += 0.001;
            System.out.println("MECHANISM " + motor.getName() + "IS STILL NOT MOVING AT SPEED: " + rampVoltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
        motor.setIdleMode(startingIdleMode);
    }

    @Override
    public boolean isFinished() {
        return movedCounter > 3;
    }
}
