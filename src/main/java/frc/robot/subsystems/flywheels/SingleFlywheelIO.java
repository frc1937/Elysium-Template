package frc.robot.subsystems.flywheels;

import frc.lib.math.Conversions;
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
        setTargetVelocityRPS(Conversions.mpsToRps(tangentialVelocity, getFlywheelDiameter()));
    }

    protected boolean hasReachedTarget() {
        double positiveDifference = Math.abs(singleFlywheelInputs.targetVelocityRotationsPerSecond -
                singleFlywheelInputs.velocityRotationsPerSecond);

        double negativeDifference = Math.abs(singleFlywheelInputs.targetVelocityRotationsPerSecond +
                singleFlywheelInputs.velocityRotationsPerSecond); //to account for inverted motors

        Logger.recordOutput("HAS REACHED? NO! + postivei" + name, positiveDifference);
        Logger.recordOutput("HAS REACHED? NO! + negative " + name, negativeDifference);

        return positiveDifference < FlywheelsConstants.TOLERANCE_ROTATIONS_PER_SECONDS || negativeDifference < FlywheelsConstants.TOLERANCE_ROTATIONS_PER_SECONDS;
    }

    protected double getFlywheelDiameter() {
        return 0;
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
