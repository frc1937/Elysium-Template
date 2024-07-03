package frc.robot.subsystems.arm.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmInputsAutoLogged;

import static frc.robot.subsystems.arm.simulation.SimulationArmConstants.ARM_MECHANISM;
import static frc.robot.subsystems.arm.simulation.SimulationArmConstants.ARM_MOTOR;

public class SimulationArm extends ArmIO {
    private Rotation2d targetAngle;

    @Override
    public void periodic() {
        if(targetAngle != null)
            ARM_MECHANISM.updateMechanism(Rotation2d.fromRotations(ARM_MOTOR.getPositionRotations()), targetAngle);
    }

    @Override
    public void setRawVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        targetAngle = targetPosition;

        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }

    @Override
    public void refreshInputs(ArmInputsAutoLogged armInputs) {
        armInputs.positionRotations = ARM_MOTOR.getPositionRotations();
        armInputs.velocityRotationsPerSecond = ARM_MOTOR.getVelocityRotationsPerSecond();
        armInputs.voltage = ARM_MOTOR.getVoltage();
    }
}
