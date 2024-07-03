package frc.robot.subsystems.swerve.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.subsystems.swerve.simulation.SimulatedSwerveConstants.GYRO;

public class SimulationSwerveIO extends SwerveIO {

    @Override
    public void setGyroHeading(Rotation2d angle) {
        GYRO.setHeading(angle);
    }

    @Override
    public void refreshInputs(SwerveInputsAutoLogged swerveInputs) {
        GYRO.update(RobotContainer.SWERVE.getSelfRelativeSpeeds().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        swerveInputs.gyroYawDegrees = GYRO.getGyroYawDegrees();

        swerveInputs.odometryUpdatesYawDegrees =  new double[]{swerveInputs.gyroYawDegrees};
        swerveInputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }
}
