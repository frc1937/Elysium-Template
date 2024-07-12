package frc.lib.generic.pigeon;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.RobotContainer.SWERVE;

public class SimulatedIMU implements Pigeon {
    private final GenericPigeonInputsAutoLogged inputs = new GenericPigeonInputsAutoLogged();
    private final String name;

    private double simulatedYawDegrees = 0;

    public SimulatedIMU(String name) {
        this.name = name;
    }

    public void update(double omegaRadiansPerSecond, double timeSeconds) {
        simulatedYawDegrees += Units.radiansToDegrees(omegaRadiansPerSecond * timeSeconds);
    }

    @Override
    public void resetConfigurations() {
    }

    @Override
    public double getYaw() {
        return simulatedYawDegrees;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public void setGyroYaw(double yawDegrees) {
        simulatedYawDegrees = yawDegrees;
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal) {
    }

    @Override
    public void periodic() {
        refreshInputs();
        Logger.processInputs(name, inputs);
    }

    @Override
    public LoggableInputs getInputs() {
        return inputs;
    }

    @Override
    public void close() {
    }

    private void refreshInputs() {
        update(SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        inputs.gyroYawDegrees = getYaw();

        inputs.odometryUpdatesYawDegrees = new double[]{inputs.gyroYawDegrees};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }
}
