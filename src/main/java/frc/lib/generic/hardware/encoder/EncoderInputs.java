package frc.lib.generic.hardware.encoder;

import frc.lib.generic.advantagekit.ChoosableLoggedInputs;
import org.littletonrobotics.junction.LogTable;

public class EncoderInputs implements ChoosableLoggedInputs {
    public static final int ENCODER_INPUTS_LENGTH = 5;

    public double position = 0;
    public double velocity = 0;

    public double[] timestamps = new double[0];
    public double[] threadPosition = new double[0];
    public double[] threadVelocity = new double[0];

    private boolean[] signalsToLog;

    @Override
    public void setSignalsToLog(boolean[] signalsToLog) {
        this.signalsToLog = signalsToLog;
    }

    @Override
    public void toLog(LogTable table) {
        if (signalsToLog == null) {
            System.out.println("ENCODER Signals to log do be null! Nudge Wihy to fix this..");
            return;
        }

        if (signalsToLog[0]) table.put("Position", position);
        if (signalsToLog[1]) table.put("Velocity", velocity);
        if (signalsToLog[2]) table.put("Timestamps", timestamps);
        if (signalsToLog[3]) table.put("ThreadPosition", threadPosition);
        if (signalsToLog[4]) table.put("ThreadVelocity", threadVelocity);
    }

    @Override
    public void fromLog(LogTable table) {
        position = table.get("Position", position);
        velocity = table.get("Velocity", velocity);
        timestamps = table.get("Timestamps", timestamps);
        threadPosition = table.get("ThreadPosition", threadPosition);
        threadVelocity = table.get("ThreadVelocity", threadVelocity);
    }
}