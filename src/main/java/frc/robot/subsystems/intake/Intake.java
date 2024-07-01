package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private final IntakeIO intakeIO = IntakeIO.generateIntake();

    public Command setIntakeSpeed(double speed) {
        return Commands.startEnd(
                () -> setPercentageOutput(speed),
                this::stop,
                this
        );
    }

    private void stop() {
        intakeIO.stop();
    }

    private void setPercentageOutput(double speed) {
        intakeIO.setPercentageOutput(speed);
    }

    @Override
    public void periodic() {
        intakeIO.refreshInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
