package frc.lib.generic.pigeon.hardware;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.pigeon.Pigeon;
import frc.lib.generic.pigeon.PigeonInputsAutoLogged;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.RobotContainer.SWERVE;

public class SimulatedIMU extends Pigeon {
    private double simulatedYawDegrees = 0;

    public SimulatedIMU(String name) {
        super(name);
    }

    public void update(double omegaRadiansPerSecond, double timeSeconds) {
        simulatedYawDegrees += Units.radiansToDegrees(omegaRadiansPerSecond * timeSeconds);
    }

    @Override
    public double getYaw() {
        return simulatedYawDegrees;
    }

    @Override
    public void setGyroYaw(double yawDegrees) {
        simulatedYawDegrees = yawDegrees;
    }

    @Override
    protected void refreshInputs(PigeonInputsAutoLogged inputs) {
        if (SWERVE == null) return; //The gyro is initialized before the swerve at the beginning

        update(SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        inputs.gyroYawDegrees = getYaw();

        inputs.odometryUpdatesYawDegrees = new double[]{inputs.gyroYawDegrees};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }
}
