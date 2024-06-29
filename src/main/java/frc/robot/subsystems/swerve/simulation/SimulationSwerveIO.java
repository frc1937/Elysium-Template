package frc.robot.subsystems.swerve.simulation;

import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class SimulationSwerveIO extends SwerveIO {

    @Override
    public void refreshInputs(SwerveInputsAutoLogged swerveInputs) {
        swerveInputs.gyroYawDegrees = 0;
        swerveInputs.gyroPitchDegrees = 0;

        swerveInputs.odometryUpdatesTimestamp = new double[0];
        swerveInputs.odometryUpdatesYawDegrees = new double[0];
    }
}
