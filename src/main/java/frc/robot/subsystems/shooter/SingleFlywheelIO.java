package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SingleFlywheelIO {
    private final SingleFlywheelInputsAutoLogged singleFlywheelInputs = new SingleFlywheelInputsAutoLogged();
    private final String name;

    public SingleFlywheelIO(String name) {
        this.name = name;
    }

    public void periodic() {
        refreshInputs(singleFlywheelInputs);
        Logger.processInputs("Flywheel/" + name + "/", singleFlywheelInputs);

        flywheelPeriodic();
    }

    protected void setTargetVelocityRPS(double velocityRPS) { }

    protected void setTargetTangentialVelocity(double tangentialVelocity) {
        setTargetVelocityRPS(tangentialVelocity / getFlywheelDiameter() * 60);
    }

    protected void flywheelPeriodic() { }
    protected void stop() { }
    protected void refreshInputs(SingleFlywheelInputsAutoLogged singleFlywheelInputs) { }

    @AutoLog
    public static class SingleFlywheelInputs {
        public double voltage = 0;
        public double velocityRotationsPerSecond = 0;
        public double targetVelocityRotationsPerSecond = 0;
        public double temperature = 0;
    }
}
