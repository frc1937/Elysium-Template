package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;

import static frc.lib.generic.hardware.pigeon.PigeonInputs.PIGEON_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public class Pigeon implements LoggableHardware {
    private final PigeonInputs inputs = new PigeonInputs();
    private final String name;

    public Pigeon(String name) {
        this.name = name;

        HardwareManager.addHardware(this);
    }

    public void configurePigeon(PigeonConfiguration pigeonConfiguration) {}

    public double getYaw() {
        if (!getSignalsToLog()[0]) printSignalError("YAW");
        return inputs.gyroYawDegrees;
    }

    public double getRoll() {
        if (!getSignalsToLog()[1]) printSignalError("ROLL");
        return inputs.gyroRollDegrees;
    }

    public double getPitch() {
        if (!getSignalsToLog()[2]) printSignalError("PITCH");
        return inputs.gyroPitchDegrees;
    }

    public void setGyroYaw(double yawDegrees) {}

    /**
     * Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.
     */
    public void setupSignalUpdates(PigeonSignal signal, boolean useFasterThread) { }

    public void setupSignalUpdates(PigeonSignal signal) { setupSignalUpdates(signal, false); }

    public boolean[] getSignalsToLog() {
        return new boolean[PIGEON_INPUTS_LENGTH];
    }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs("Pigeons/" + name, inputs);
    }

    @Override
    public PigeonInputs getInputs() {
        return inputs;
    }
    
    protected void refreshInputs(PigeonInputs inputs) {}

    private void printSignalError(String signalName) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) return;

        new NoSuchElementException("--------------\n" +
                "ERROR - TRYING TO RETRIEVE UNINITIALIZED SIGNAL " + signalName + "| AT " + getClass().getName() + name +
                "\n--------------")
                .printStackTrace();
    }
}
