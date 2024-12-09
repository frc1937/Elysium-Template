package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.commands.ExecuteEndCommand;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR;

public class Intake extends GenericSubsystem {
    public Command setIntakeVoltage(double voltage) {
        return new ExecuteEndCommand(
                () -> setVoltageOutput(voltage),
                this::stop,
                this
        );
    }

    private void setVoltageOutput(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    private void stop() {
        INTAKE_MOTOR.stopMotor();
    }
}
