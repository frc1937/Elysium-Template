// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.poseestimation.poseestimator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.GlobalConstants.ODOMETRY_LOCK;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock = new ReentrantLock();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];

    private static PhoenixOdometryThread INSTANCE = null;

    public static PhoenixOdometryThread getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PhoenixOdometryThread();
        }
        return INSTANCE;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }

    public Queue<Double> getTimestampQueue() {
        return timestamps;
    }

    public Queue<Double> registerSignal(StatusSignal<Double> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        signalsLock.lock();
        ODOMETRY_LOCK.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);
        } finally {
            signalsLock.unlock();
            ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        Timer.delay(5);
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY_HERTZ));
                if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }
            double fpgaTimestamp = Logger.getRealTimestamp() / 1.0e6;

            // Save new data to queues
            ODOMETRY_LOCK.lock();
            try {
                for (int i = 0; i < signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
                timestamps.offer(fpgaTimestamp);
            } finally {
                ODOMETRY_LOCK.unlock();
            }
        }
    }
}