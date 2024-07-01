package frc.robot.subsystems.kicker;

import frc.robot.GlobalConstants;
import frc.robot.subsystems.kicker.real.RealKicker;
import frc.robot.subsystems.kicker.simulation.SimulatedKicker;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class KickerIO {

    static KickerIO generateKickerIO() {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
            return new RealKicker();
        }

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedKicker();
        }

        return new KickerIO();
    }


    protected void refreshInputs(KickerInputsAutoLogged inputs) {
    }

    protected void setPercentageOutput(double percentageOutput) {
    }

    protected void stop() {
    }

    @AutoLog
    public static class KickerInputs {
        public double voltage = 0;
        public boolean doesSeeNote = false;
    }
}
