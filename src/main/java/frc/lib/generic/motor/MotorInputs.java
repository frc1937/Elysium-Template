package frc.lib.generic.motor;

import frc.lib.generic.advantagekit.ChoosableLoggedInputs;
import org.littletonrobotics.junction.LogTable;

import static frc.lib.generic.motor.Motor.LOG_NO_SIGNALS;

public class MotorInputs implements ChoosableLoggedInputs {
    public double voltage = 0;
    public double current = 0;
    public double temperature = 0;
    public double target = 0;
    public double systemPosition = 0;
    public double systemVelocity = 0;

    public double[] timestamps = new double[0];
    public double[] threadVoltage = new double[0];
    public double[] threadCurrent = new double[0];
    public double[] threadTemperature = new double[0];
    public double[] threadTarget = new double[0];
    public double[] threadSystemPosition = new double[0];
    public double[] threadSystemVelocity = new double[0];

    private boolean[] signalsToLog = LOG_NO_SIGNALS;

    @Override
    public void setSignalsToLog(boolean[] signalsToLog) {
        this.signalsToLog = signalsToLog;
    }

    @Override
    public void toLog(LogTable table) {
        if (signalsToLog[0]) table.put("Voltage", voltage);
        if (signalsToLog[1]) table.put("Current", current);
        if (signalsToLog[2]) table.put("Temperature", temperature);
        if (signalsToLog[3]) table.put("Target", target);
        if (signalsToLog[4]) table.put("SystemPosition", systemPosition);
        if (signalsToLog[5]) table.put("SystemVelocity", systemVelocity);
        if (signalsToLog[6]) table.put("Timestamps", timestamps);
        if (signalsToLog[7]) table.put("ThreadVoltage", threadVoltage);
        if (signalsToLog[8]) table.put("ThreadCurrent", threadCurrent);
        if (signalsToLog[9]) table.put("ThreadTemperature", threadTemperature);
        if (signalsToLog[10]) table.put("ThreadTarget", threadTarget);
        if (signalsToLog[11]) table.put("ThreadSystemPosition", threadSystemPosition);
        if (signalsToLog[12]) table.put("ThreadSystemVelocity", threadSystemVelocity);
    }

    @Override
    public void fromLog(LogTable table) {
//        if (signalsToLog[0])
            voltage = table.get("Voltage", voltage);
//        if (signalsToLog[1])
            current = table.get("Current", current);
//        if (signalsToLog[2])
            temperature = table.get("Temperature", temperature);
//        if (signalsToLog[3])
            target = table.get("Target", target);
//        if (signalsToLog[4])
            systemPosition = table.get("SystemPosition", systemPosition);
//        if (signalsToLog[5])
            systemVelocity = table.get("SystemVelocity", systemVelocity);
//        if (signalsToLog[6])
            timestamps = table.get("Timestamps", timestamps);
//        if (signalsToLog[7])
            threadVoltage = table.get("ThreadVoltage", threadVoltage);
//        if (signalsToLog[8])
            threadCurrent = table.get("ThreadCurrent", threadCurrent);
//        if (signalsToLog[9])
            threadTemperature = table.get("ThreadTemperature", threadTemperature);
//        if (signalsToLog[10])
            threadTarget = table.get("ThreadTarget", threadTarget);
//        if (signalsToLog[11])
            threadSystemPosition = table.get("ThreadSystemPosition", threadSystemPosition);
        if (signalsToLog[12]) threadSystemVelocity = table.get("ThreadSystemVelocity", threadSystemVelocity);
    }
}
