package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimation.poseestimator.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

import static frc.robot.subsystems.swerve.real.RealSwerveConstants.GYRO;

public class RealSwerve extends SwerveIO {
    private final Queue<Double>
            yawQueue = SparkOdometryThread.getInstance().registerSignal(GYRO.get()::getYaw),
            timestampQueue = SparkOdometryThread.getInstance().getTimestampQueue();

    @Override
    protected void setGyroHeading(Rotation2d heading) {
        GYRO.get().setYaw(heading.getDegrees());
    }

    @Override
    protected void refreshInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = GYRO.get().getYaw();

        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }
}
