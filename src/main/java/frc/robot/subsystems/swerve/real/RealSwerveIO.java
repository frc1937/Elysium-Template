package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimation.poseestimator.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

import static frc.robot.subsystems.swerve.real.RealSwerveConstants.GYRO;

public class RealSwerveIO extends SwerveIO {
    private final WPI_PigeonIMU gyro = GYRO.get();
    private final Queue<Double>
            yawQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getYaw),
            timestampQueue = SparkOdometryThread.getInstance().getTimestampQueue();

    @Override
    protected void setGyroHeading(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }

    @Override
    protected void refreshInputs(SwerveInputsAutoLogged swerveInputs) {
        swerveInputs.gyroYawDegrees = gyro.getYaw();

        swerveInputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        swerveInputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }
}
