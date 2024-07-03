package frc.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.simulation.GyroSimulation;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.RobotContainer.SWERVE;

public class SimulationSwerve extends SwerveIO {
    private final GyroSimulation gyro = SimulationSwerveConstants.GYRO;

    @Override
    protected void setGyroHeading(Rotation2d heading) {
        gyro.setHeading(heading);
    }

    @Override
    protected void refreshInputs(SwerveInputsAutoLogged inputs) {
        gyro.update(SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        inputs.gyroYawDegrees = gyro.getGyroYawDegrees();

        inputs.odometryUpdatesYawDegrees = new double[]{inputs.gyroYawDegrees};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }
}
