package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.commands.ExecuteEndCommand;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR;

public class Intake extends GenericSubsystem {
    public Command setIntakeSpeedPercentage(double speedPercentage) {
        return new ExecuteEndCommand(
                () -> setPercentageOutput(speedPercentage),
                this::stop,
                this
        );
    }

    private void setPercentageOutput(double percentageOutput) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentageOutput);
    }

    private void stop() {
        INTAKE_MOTOR.stopMotor();
    }
}
