package frc.robot.subsystems.intake;

import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.real.RealIntake;
import frc.robot.subsystems.intake.simulation.SimulatedIntake;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class IntakeIO {
    static IntakeIO generateIntake() {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL)
            return new RealIntake();

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulatedIntake();

        return new IntakeIO();
    }

    protected void stop() { }

    protected void setPercentageOutput(double percentage) { }

    protected void refreshInputs(IntakeInputsAutoLogged intakeInputs) { }

    @AutoLog
    public static class IntakeInputs {
        public double voltage = 0;
    }
}
