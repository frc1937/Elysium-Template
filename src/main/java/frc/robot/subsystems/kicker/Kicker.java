package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final KickerIO kickerIO = KickerIO.generateKickerIO();
    private final KickerInputsAutoLogged kickerInputs = new KickerInputsAutoLogged();

    public Command setKickerPercentageOutput(double percentageOutput) {
        return Commands.startEnd(
                () -> kickerIO.setPercentageOutput(percentageOutput),
                this::stop,
                this
        );
    }

    public boolean doesSeeNote() {
        return kickerInputs.doesSeeNote;
    }

    private void stop() {
        kickerIO.stop();
    }

    @Override
    public void periodic() {
        kickerIO.refreshInputs(kickerInputs);
        Logger.processInputs("Kicker", kickerInputs);
    }
}
