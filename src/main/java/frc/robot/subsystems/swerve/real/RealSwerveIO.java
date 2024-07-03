package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimation.poseestimator.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

public class RealSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = RealSwerveConstants.GYRO.get();

    private final Queue<Double>
            yawQueue = PhoenixOdometryThread.getInstance().registerSignal(RealSwerveConstants.YAW_SIGNAL),
            timestampQueue = PhoenixOdometryThread.getInstance().getTimestampQueue();

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.gyroYawDegrees = RealSwerveConstants.YAW_SIGNAL.getValue();

        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                RealSwerveConstants.YAW_SIGNAL
        );
    }
}
