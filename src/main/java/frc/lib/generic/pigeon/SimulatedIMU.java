package frc.lib.generic.pigeon;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.advantagekit.HardwareManager;
import org.littletonrobotics.junction.Logger;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.RobotContainer.SWERVE;

public class SimulatedIMU extends Pigeon {
    private final PigeonInputsAutoLogged inputs = new PigeonInputsAutoLogged();
    private final String name;

    private double simulatedYawDegrees = 0;

    public SimulatedIMU(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
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
    public void periodic() {
        refreshInputs();
        Logger.processInputs(name, inputs);
    }

    @Override
    public PigeonInputsAutoLogged getInputs() {
        return inputs;
    }

    private void refreshInputs() {
        update(SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        inputs.gyroYawDegrees = getYaw();

        inputs.odometryUpdatesYawDegrees = new double[]{inputs.gyroYawDegrees};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }
}
