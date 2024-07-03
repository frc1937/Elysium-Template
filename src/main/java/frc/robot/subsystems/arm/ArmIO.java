package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.arm.real.RealArm;
import frc.robot.subsystems.arm.simulation.SimulationArm;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    static ArmIO generateArm() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return new RealArm();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulationArm();

        return new ArmIO();
    }

    public void periodic() {}

    protected void setRawVoltage(double voltage) { }
    protected void setTargetPosition(Rotation2d targetPosition) { }

    protected boolean hasReachedTarget() { return false; }

    protected void refreshInputs(ArmInputsAutoLogged armInputs) { }

    @AutoLog
    public static class ArmInputs {
        public double positionRotations = 0;
        public double velocityRotationsPerSecond = 0;

        public double voltage = 0;
    }
}
