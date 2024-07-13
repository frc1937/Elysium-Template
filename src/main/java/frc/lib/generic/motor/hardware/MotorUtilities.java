package frc.lib.generic.motor.hardware;

import frc.lib.generic.motor.MotorInputsAutoLogged;
import frc.lib.generic.simulation.GenericSimulation;
import frc.robot.GlobalConstants;

import java.util.Map;
import java.util.Queue;

import static frc.robot.GlobalConstants.CURRENT_MODE;

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

    static boolean handleSimulationInputs(MotorInputsAutoLogged inputs, GenericSimulation simulation) {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            inputs.systemPosition = simulation.getPositionRotations();
            inputs.systemVelocity = simulation.getVelocityRotationsPerSecond();
            inputs.voltage = simulation.getVoltage();
            inputs.current = simulation.getCurrent();

            inputs.temperature = 0.0;
            inputs.target = simulation.getTarget();

            inputs.threadSystemPosition = new double[] {inputs.systemPosition};
            inputs.threadSystemVelocity = new double[] {inputs.systemVelocity};
            inputs.threadVoltage = new double[] {inputs.voltage};
            inputs.threadCurrent = new double[] {inputs.current};
            inputs.threadTemperature = new double[] {inputs.temperature};
            inputs.threadTarget = new double[] {inputs.target};

            return true;
        }

        return false;
    }
}
