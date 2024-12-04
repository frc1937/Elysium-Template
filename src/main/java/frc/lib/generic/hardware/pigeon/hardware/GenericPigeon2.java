package frc.lib.generic.hardware.pigeon.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonInputs;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import static frc.lib.generic.hardware.pigeon.PigeonInputs.PIGEON_INPUTS_LENGTH;
import static frc.lib.generic.hardware.pigeon.hardware.PigeonUtilities.handleThreadedInputs;

public class GenericPigeon2 extends Pigeon {
    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yawSignal, pitchSignal, rollSignal;
    private final List<StatusSignal<Double>> signalsToUpdateList = new ArrayList<>();

    private final boolean[] signalsToLog = new boolean[PIGEON_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    public GenericPigeon2(String name, int deviceNumber, String canbusName) {
        super(name);

        pigeon = new Pigeon2(deviceNumber, canbusName);

        yawSignal = pigeon.getYaw().clone();
        pitchSignal = pigeon.getPitch().clone();
        rollSignal = pigeon.getRoll().clone();
    }

    public GenericPigeon2(String name, int deviceNumber) {
        this(name, deviceNumber, "CAN");
    }

    @Override
    public void configurePigeon(PigeonConfiguration pigeonConfiguration) {
        pigeon.reset();

        final Pigeon2Configuration configuration  = new Pigeon2Configuration();
        final Rotation3d centerOfRotationOffset = pigeonConfiguration.centerOfRotationOffset;

        configuration.MountPose.MountPoseYaw = Units.radiansToDegrees(centerOfRotationOffset.getZ());
        configuration.MountPose.MountPosePitch = Units.radiansToDegrees(centerOfRotationOffset.getY());
        configuration.MountPose.MountPoseRoll = Units.radiansToDegrees(centerOfRotationOffset.getX());

        pigeon.optimizeBusUtilization();

        pigeon.getConfigurator().apply(configuration);
    }

    @Override
    public void setGyroYaw(double yawDegrees) {
        pigeon.setYaw(yawDegrees);
    }

    @Override
    public boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal, boolean useFasterThread) {
        final int updateFrequency = useFasterThread ? 200 : 50;
        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case YAW -> setupSignal(yawSignal, updateFrequency);
            case ROLL -> setupSignal(rollSignal, updateFrequency);
            case PITCH -> setupSignal(pitchSignal, updateFrequency);
        }

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + PIGEON_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case YAW -> signalQueueList.put("yaw", OdometryThread.getInstance().registerSignal(this::getYawPrivate));
            case ROLL -> signalQueueList.put("roll", OdometryThread.getInstance().registerSignal(this::getRollPrivate));
            case PITCH -> signalQueueList.put("pitch", OdometryThread.getInstance().registerSignal(this::getPitchPrivate));
        }
    }

    @Override
    protected void refreshInputs(PigeonInputs inputs) {
        if (pigeon == null) return;

        inputs.setSignalsToLog(signalsToLog);

        BaseStatusSignal.refreshAll(signalsToUpdateList.toArray(new BaseStatusSignal[0]));

        inputs.gyroYawDegrees = getYawPrivate();
        inputs.gyroRollDegrees = getRollPrivate();
        inputs.gyroPitchDegrees = getPitchPrivate();

        handleThreadedInputs(inputs, signalQueueList);
    }

    private double getYawPrivate() {
        return yawSignal.getValue();
    }

    private double getPitchPrivate() {
        return pitchSignal.getValue();
    }

    private double getRollPrivate() {
        return rollSignal.getValue();
    }

    private void setupSignal(final StatusSignal<Double> correspondingSignal, final int updateFrequency) {
        correspondingSignal.setUpdateFrequency(updateFrequency);
        signalsToUpdateList.add(correspondingSignal);
    }
}
