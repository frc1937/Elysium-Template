package frc.lib.generic.hardware.pigeon.hardware;

import frc.lib.generic.hardware.pigeon.PigeonInputs;

import java.util.Map;
import java.util.Queue;

public class PigeonUtilities {
    public static void handleThreadedInputs(PigeonInputs inputs, Map<String, Queue<Double>> signalQueueList) {
        if (signalQueueList.isEmpty()) return;

        if (signalQueueList.get("yaw") != null)
            inputs.threadGyroYawDegrees = signalQueueList.get("yaw").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("pitch") != null)
            inputs.threadGyroPitchDegrees = signalQueueList.get("pitch").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("roll") != null)
            inputs.threadGyroRollDegrees = signalQueueList.get("roll").stream().mapToDouble(Double::doubleValue).toArray();

        signalQueueList.forEach((k, v) -> v.clear());
    }
}
