package frc.robot.subsystems.shooter;

import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.real.RealFlywheels;
import frc.robot.subsystems.shooter.simulation.SimulatedFlywheels;
import org.littletonrobotics.junction.AutoLog;

public class FlywheelsIO {
    static FlywheelsIO generateIO() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new FlywheelsIO();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulatedFlywheels();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return new RealFlywheels();

        return new FlywheelsIO();
    }

    protected void refreshInputs(FlywheelsInputsAutoLogged flywheelsInputs) {
    }

    @AutoLog
    protected static class FlywheelsInputs {
        public double currentTangentialVelocity = 0;
    }
}
