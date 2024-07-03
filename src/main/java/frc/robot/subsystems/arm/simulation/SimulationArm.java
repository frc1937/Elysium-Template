package frc.robot.subsystems.arm.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmInputsAutoLogged;

import static frc.robot.subsystems.arm.ArmConstants.TOLERANCE_ROTATIONS;
import static frc.robot.subsystems.arm.simulation.SimulationArmConstants.ARM_MECHANISM;
import static frc.robot.subsystems.arm.simulation.SimulationArmConstants.ARM_MOTOR;

public class SimulationArm extends ArmIO {
    private Rotation2d targetPosition;

    @Override
    public void periodic() {
        if(targetPosition != null)
            ARM_MECHANISM.updateMechanism(Rotation2d.fromRotations(ARM_MOTOR.getPositionRotations()), targetPosition);
    }

    @Override
    public void setRawVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        this.targetPosition = targetPosition;

        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }

    @Override
    public boolean hasReachedTarget() {
        return Math.abs(ARM_MOTOR.getPositionRotations() - targetPosition.getRotations()) < TOLERANCE_ROTATIONS;
    }

    @Override
    public void refreshInputs(ArmInputsAutoLogged armInputs) {
        armInputs.positionRotations = ARM_MOTOR.getPositionRotations();
        armInputs.velocityRotationsPerSecond = ARM_MOTOR.getVelocityRotationsPerSecond();
        armInputs.voltage = ARM_MOTOR.getVoltage();
    }
}
