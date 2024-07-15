package frc.lib.generic.motor.hardware;

import frc.lib.generic.motor.MotorInputsAutoLogged;

import java.util.Map;
import java.util.Queue;

public class MotorUtilities {
    static void handleThreadedInputs(MotorInputsAutoLogged inputs, Map<String, Queue<Double>> signalQueueList, Queue<Double> timestampQueue) {
        if (signalQueueList.isEmpty()) return;

        inputs.threadSystemPosition = signalQueueList.get("position").stream().mapToDouble(Double::doubleValue).toArray();
        inputs.threadSystemVelocity = signalQueueList.get("velocity").stream().mapToDouble(Double::doubleValue).toArray();
        inputs.threadVoltage = signalQueueList.get("voltage").stream().mapToDouble(Double::doubleValue).toArray();
        inputs.threadCurrent = signalQueueList.get("current").stream().mapToDouble(Double::doubleValue).toArray();
        inputs.threadTemperature = signalQueueList.get("temperature").stream().mapToDouble(Double::doubleValue).toArray();
        inputs.threadTarget = signalQueueList.get("target").stream().mapToDouble(Double::doubleValue).toArray();

        inputs.timestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        signalQueueList.forEach((k, v) -> v.clear());
        timestampQueue.clear();
    }
}
