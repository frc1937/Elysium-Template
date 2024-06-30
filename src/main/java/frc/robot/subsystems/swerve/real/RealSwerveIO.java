package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.threads.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

import static frc.robot.subsystems.swerve.real.RealSwerveConstants.GYRO;

public class RealSwerveIO extends SwerveIO {
    Queue<Double> signalQueue, timestampQueue;

    public RealSwerveIO() {
        signalQueue = SparkOdometryThread.getInstance().registerSignal(GYRO::getYaw);
        timestampQueue = SparkOdometryThread.getInstance().getTimestamps();
    }

    @Override
    protected void setGyroHeading(Rotation2d angle) {
        GYRO.setYaw(angle.getDegrees());
    }

    @Override
    protected void refreshInputs(SwerveInputsAutoLogged swerveInputs) {
        swerveInputs.gyroYawDegrees = GYRO.getYaw();

        swerveInputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        swerveInputs.odometryUpdatesYawDegrees = signalQueue.stream().mapToDouble(Double::doubleValue).toArray();

        signalQueue.clear();
        timestampQueue.clear();
    }
}
