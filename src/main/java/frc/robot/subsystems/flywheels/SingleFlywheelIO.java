package frc.robot.subsystems.flywheels;

import frc.lib.generic.simulation.mechanisms.SpeedMechanism2d;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SingleFlywheelIO {
    private final SingleFlywheelInputsAutoLogged singleFlywheelInputs = new SingleFlywheelInputsAutoLogged();

    private final SpeedMechanism2d speedMechanism2d;
    private final String name;
    private final double flywheelDiameter;

    public SingleFlywheelIO(String name, double flywheelDiameter, boolean inverted) {
        this.name = name;
        this.flywheelDiameter = flywheelDiameter;

        speedMechanism2d = new SpeedMechanism2d("Flywheel" + name, FlywheelsConstants.MAXIMUM_VELOCITY_RPM / 60, inverted);
    }

    public String getName() {
        return name;
    }

    public void periodic() {
        refreshInputs(singleFlywheelInputs);
        Logger.processInputs("Flywheel/" + name + "/", singleFlywheelInputs);

        speedMechanism2d.updateMechanism(singleFlywheelInputs.velocityRotationsPerSecond, singleFlywheelInputs.targetVelocityRotationsPerSecond);
    }

    protected void setTargetVelocity(double velocityRPS) { }

    protected void setTargetTangentialVelocity(double tangentialVelocity) {
        setTargetVelocity(Conversions.mpsToRps(tangentialVelocity, flywheelDiameter));
    }

    protected void setRawVoltage(double voltage) { }
    protected double getVoltage() { return singleFlywheelInputs.voltage; }
    protected double getVelocityRotationsPerSecond() { return singleFlywheelInputs.velocityRotationsPerSecond; }

    protected boolean hasReachedTarget() {
        double difference = Math.min(
                Math.abs(singleFlywheelInputs.targetVelocityRotationsPerSecond - singleFlywheelInputs.velocityRotationsPerSecond),
                Math.abs(singleFlywheelInputs.targetVelocityRotationsPerSecond + singleFlywheelInputs.velocityRotationsPerSecond)
        );

        return difference < FlywheelsConstants.TOLERANCE_ROTATIONS_PER_SECONDS;
    }

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
