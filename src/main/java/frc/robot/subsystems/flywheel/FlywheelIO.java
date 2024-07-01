package frc.robot.subsystems.flywheel;

import frc.robot.GlobalConstants;
import frc.robot.subsystems.flywheel.real.RealFlywheelConstants;
import frc.robot.subsystems.flywheel.simulation.SimulatedFlywheelConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class FlywheelIO {
    private final FlywheelInputsAutoLogged flywheelInputs = new FlywheelInputsAutoLogged();
    private final String name;

    static FlywheelIO[] generateFlywheels() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return RealFlywheelConstants.getFlywheels();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return SimulatedFlywheelConstants.getFlywheels();

        return FlywheelConstants.getFlywheels();
    }

    public FlywheelIO(String name) {
        this.name = name;
    }

    public void periodic() {
        refreshInputs(flywheelInputs);
        Logger.processInputs("Flywheel/" + name + "/", flywheelInputs);
    }

    public void setTargetVelocity(double velocityRotationsPerSecond) {}

    public void refreshInputs(FlywheelInputsAutoLogged flywheelInputs) {}

    @AutoLog
    public static class FlywheelInputs {
        public double velocityRotationsPerSecond = 0;
        public double voltage = 0;
        public double temperature = 0;
    }
}
