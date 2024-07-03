package frc.lib.util.threads;

import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.GlobalConstants.ODOMETRY_LOCK;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkOdometryThread {
    private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
    private List<Queue<Double>> queues = new ArrayList<>();
    private List<Queue<Double>> timestampQueues = new ArrayList<>();

    private final Notifier notifier;
    private static SparkOdometryThread instance = null;

    public static SparkOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkOdometryThread();
        }
        return instance;
    }

    private SparkOdometryThread() {
        notifier = new Notifier(this::periodic);
        notifier.setName("SparkMaxOdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
        }
    }

    public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        ODOMETRY_LOCK.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        ODOMETRY_LOCK.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    private void periodic() {
        ODOMETRY_LOCK.lock();
        double timestamp = Logger.getRealTimestamp() / 1e6;
        try {
            double[] values = new double[signals.size()];
            boolean isValid = true;
            for (int i = 0; i < signals.size(); i++) {
                OptionalDouble value = signals.get(i).get();
                if (value.isPresent()) {
                    values[i] = value.getAsDouble();
                } else {
                    isValid = false;
                    break;
                }
            }
            if (isValid) {
                for (int i = 0; i < queues.size(); i++) {
                    queues.get(i).offer(values[i]);
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            }
        } finally {
            ODOMETRY_LOCK.unlock();
        }
    }
}