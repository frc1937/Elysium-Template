package frc.lib.generic.pigeon.hardware;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.lib.generic.pigeon.Pigeon;
import frc.lib.generic.pigeon.PigeonInputsAutoLogged;
import frc.lib.generic.pigeon.PigeonSignal;
import frc.robot.poseestimation.poseestimator.OdometryThread;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

public class GenericIMU extends Pigeon {
    private final WPI_PigeonIMU pigeon;

    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();
    private final Queue<Double> timestampQueue = OdometryThread.getInstance().getTimestampQueue();

    public GenericIMU(String name, int deviceNumber) {
        super(name);

        pigeon = new WPI_PigeonIMU(deviceNumber);
    }

    @Override
    public void resetConfigurations() {
        pigeon.configFactoryDefault();
    }

    @Override
    public void setGyroYaw(double yawDegrees) {
        pigeon.setYaw(yawDegrees);
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal) {
        if (!signal.useFasterThread()) return;

        switch (signal.getType()) {
            case YAW -> signalQueueList.put("yaw", OdometryThread.getInstance().registerSignal(pigeon::getYaw));
            case ROLL -> signalQueueList.put("roll", OdometryThread.getInstance().registerSignal(pigeon::getRoll));
            case PITCH -> signalQueueList.put("pitch", OdometryThread.getInstance().registerSignal(pigeon::getPitch));
        }
    }//todo: Map out the structure and try to find inconsistencies.

    @Override
    protected void refreshInputs(PigeonInputsAutoLogged inputs) {
        if (pigeon == null) return;

        inputs.gyroYawDegrees = pigeon.getYaw();
        inputs.gyroRollDegrees = pigeon.getRoll();
        inputs.gyroPitchDegrees = pigeon.getPitch();

        if (signalQueueList.isEmpty()) return;

        if(signalQueueList.get("yaw") != null)
            inputs.threadGyroYawDegrees = signalQueueList.get("yaw").stream().mapToDouble(Double::doubleValue).toArray();

        if(signalQueueList.get("pitch") != null)
            inputs.threadGyroPitchDegrees = signalQueueList.get("pitch").stream().mapToDouble(Double::doubleValue).toArray();

        if(signalQueueList.get("roll") != null)
            inputs.threadGyroRollDegrees = signalQueueList.get("roll").stream().mapToDouble(Double::doubleValue).toArray();

        inputs.timestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        timestampQueue.clear();
        signalQueueList.forEach((k, v) -> v.clear());
    }
}
