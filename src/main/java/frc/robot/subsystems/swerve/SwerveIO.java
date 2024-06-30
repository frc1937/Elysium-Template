package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.real.RealSwerveIO;
import frc.robot.subsystems.swerve.simulation.SimulationSwerveIO;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {
    static SwerveIO generateSwerve() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL) {
            return new RealSwerveIO();
        }

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulationSwerveIO();
        }

        return new SwerveIO();
    }

    protected void refreshInputs(SwerveInputsAutoLogged swerveInputs) {
    }

    protected void setGyroHeading(Rotation2d angle) {
    }

    @AutoLog
    protected static class SwerveInputs {
        public double gyroYawDegrees = 0;

        public double[] odometryUpdatesTimestamp = new double[0];
        public double[] odometryUpdatesYawDegrees = new double[0];
    }
}
