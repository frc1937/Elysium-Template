package frc.robot.subsystems.shooter.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.shooter.ArmIO;

import static frc.robot.subsystems.shooter.simulation.SimulationArmConstants.ARM_MECHANISM;
import static frc.robot.subsystems.shooter.simulation.SimulationArmConstants.ARM_MOTOR;

public class SimulationArm extends ArmIO {
    private Rotation2d targetAngle;

    @Override
    public void periodic() {
        if(targetAngle != null)
            ARM_MECHANISM.updateMechanism(Rotation2d.fromRotations(ARM_MOTOR.getPositionRotations()), targetAngle);
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        targetAngle = targetPosition;

        ARM_MOTOR.setInput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }

    @Override
    public void refreshInputs(ArmInputsAutoLogged armInputs) {
        armInputs.positionRotations = ARM_MOTOR.getPositionRotations();
        armInputs.velocityRotationsPerSecond = ARM_MOTOR.getVelocityRotationsPerSecond();
        armInputs.voltage = ARM_MOTOR.getVoltage();
    }
}
