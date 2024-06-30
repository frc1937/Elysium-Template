package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.real.RealArm;
import frc.robot.subsystems.shooter.simulation.SimulationArm;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {

    static ArmIO generateArm() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return new RealArm();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return new SimulationArm();

        return new ArmIO();
    }

    public void periodic() { }

    public void setTargetPosition(Rotation2d targetPosition) { }
    public void refreshInputs(ArmInputsAutoLogged armInputs) { }

    @AutoLog
    public static class ArmInputsAutoLogged {
        public double positionRotations = 0;
        public double velocityRotationsPerSecond = 0;

        public double voltage = 0;
    }
}
