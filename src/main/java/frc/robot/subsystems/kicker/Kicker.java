package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.kicker.KickerConstants.BEAM_BREAKER;
import static frc.robot.subsystems.kicker.KickerConstants.MOTOR;

public class Kicker extends GenericSubsystem {
    public Command setKickerVoltage(double voltage) {
        return Commands.startEnd(
                () -> setVoltageOutput(voltage),
                this::stop,
                this
        );
    }

    public boolean doesSeeNote() {
        return BEAM_BREAKER.get() == 0;
    }

    private void setVoltageOutput(double voltage) {
        MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    private void stop() {
        MOTOR.stopMotor();
    }
}
