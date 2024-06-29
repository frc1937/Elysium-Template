package frc.lib.util.threads;

import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.GlobalConstants.ODOMETRY_LOCK;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkOdometryThread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    private static SparkOdometryThread instance = null;

    public static SparkOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkOdometryThread();
        }
        return instance;
    }

    private SparkOdometryThread() {
        try(Notifier notifier = new Notifier(this::periodic)) {
            notifier.setName("SparkMaxOdometryThread");
            notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
        }
    }

    public Queue<Double> getTimestamps() {
        return timestamps;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        ODOMETRY_LOCK.lock();

        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            ODOMETRY_LOCK.unlock();
        }

        return queue;
    }

    private void periodic() {
        ODOMETRY_LOCK.lock();
        timestamps.offer(Logger.getRealTimestamp() / 1.0e6);

        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            ODOMETRY_LOCK.unlock();
        }
    }
}