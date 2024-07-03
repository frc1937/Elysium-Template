package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.real.RealSwerveIO;
import frc.robot.subsystems.swerve.simulationswerve.SimulationSwerveIO;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {
    static SwerveIO generateIO() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new SwerveIO();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulationSwerveIO();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return new RealSwerveIO();

        return new SwerveIO();
    }

    protected void refreshInputs(SwerveInputsAutoLogged inputs) {
    }

    protected void setGyroHeading(Rotation2d heading) {
    }

    @AutoLog
    protected static class SwerveInputs {
        public double gyroYawDegrees = 0;

        public double[] odometryUpdatesTimestamp = new double[0];
        public double[] odometryUpdatesYawDegrees = new double[0];
    }
}
