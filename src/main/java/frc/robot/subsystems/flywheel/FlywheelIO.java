package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class FlywheelIO {
    private final FlywheelInputsAutoLogged flywheelInputs = new FlywheelInputsAutoLogged();
    private final String name;

    public FlywheelIO(String name) {
        this.name = name;
    }

    public void periodic() {
        refreshInputs(flywheelInputs);
        Logger.processInputs("Flywheel/" + name + "/", flywheelInputs);

        flywheelPeriodic();
    }

    protected double getFlywheelDiameter() {
        return 0;
    }

    protected void stop() {
    }

    protected void flywheelPeriodic() {
    }

    protected boolean hasReachedTarget() {
        return false;
    }

    protected void setRawVoltage(double voltage) {
    }

    protected void setTargetVelocity(double velocityRotationsPerSecond) {
    }

    protected void refreshInputs(FlywheelInputsAutoLogged flywheelInputs) {
    }

    protected String getName() {
        return name;
    }

    protected double getVoltage() {
        return flywheelInputs.voltage;
    }

    protected double getVelocityRotationsPerSecond() {
        return flywheelInputs.velocityRotationsPerSecond;
    }


    @AutoLog
    public static class FlywheelInputs {
        public double velocityRotationsPerSecond = 0;
        public double voltage = 0;
        public double temperature = 0;
    }
}
