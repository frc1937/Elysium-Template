package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class SwerveModuleIO {
    private final SwerveModuleInputsAutoLogged swerveModuleInputs = new SwerveModuleInputsAutoLogged();

    private final String name;

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    public String getLoggingPath() {
        return "Swerve/" + name + "/";
    }

    public void periodic() {
        updateInputs(swerveModuleInputs);
        Logger.processInputs(getLoggingPath(), swerveModuleInputs);
    }

    void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getCurrentAngle());

        setTargetAngle(optimizedState.angle);
        setTargetVelocity(optimizedState.speedMetersPerSecond);
    }

    protected abstract void setTargetAngle(Rotation2d angle);
    protected abstract void setTargetVelocity(double velocityMetresPerSecond);


    protected abstract Rotation2d getCurrentAngle();

    protected abstract void updateInputs(SwerveModuleInputsAutoLogged swerveModuleInputs);

    @AutoLog
    public static class SwerveModuleInputs {
        public double steerAngleDegrees = 0;
        public double steerVoltage = 0;

        public double driveVelocityMetersPerSecond = 0;
        public double driveDistanceMeters = 0;
        public double driveVoltage = 0;

        public double[] odometryUpdatesDriveDistanceMeters = new double[0];
        public double[] odometryUpdatesSteerAngleDegrees = new double[0];
    }
}
