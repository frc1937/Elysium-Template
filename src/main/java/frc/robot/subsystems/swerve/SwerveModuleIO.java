package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIO {
    private final SwerveModuleInputsAutoLogged swerveModuleInputs = new SwerveModuleInputsAutoLogged();
    private final String name;

    private SwerveModuleState targetState  = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    public String getLoggingPath() {
        return "Swerve/" + name + "/";
    }

    public void periodic() {
        refreshInputs(swerveModuleInputs);
        Logger.processInputs(getLoggingPath(), swerveModuleInputs);
    }

    void setTargetState(SwerveModuleState state) {
        this.targetState = SwerveModuleState.optimize(state, getCurrentAngle());

        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(swerveModuleInputs.steerAngleDegrees);
    }

    protected SwerveModuleState getTargetState() {
        return targetState;
    }

    protected void setTargetAngle(Rotation2d angle) {}
    protected void setTargetVelocity(double velocityMetresPerSecond) {}

    protected void stop() {}

    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(swerveModuleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    protected void refreshInputs(SwerveModuleInputsAutoLogged swerveModuleInputs) {}

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
