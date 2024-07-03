package frc.robot.subsystems.arm.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmInputsAutoLogged;

import static frc.robot.subsystems.arm.ArmConstants.TOLERANCE_ROTATIONS;
import static frc.robot.subsystems.arm.real.RealArmConstants.ABSOLUTE_ARM_ENCODER;
import static frc.robot.subsystems.arm.real.RealArmConstants.ARM_MOTOR;

public class RealArm extends ArmIO {
    private Rotation2d targetPosition = new Rotation2d();

    @Override
    public void periodic() {
        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        this.targetPosition = targetPosition;

        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,
                targetPosition.getRotations());
    }

    @Override
    public boolean hasReachedTarget() {
        return Math.abs(ABSOLUTE_ARM_ENCODER.getEncoderPosition() - targetPosition.getRotations()) < TOLERANCE_ROTATIONS;
    }

    @Override
    public void setRawVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void refreshInputs(ArmInputsAutoLogged armInputs) {
        armInputs.positionRotations = ABSOLUTE_ARM_ENCODER.getEncoderPosition();
        armInputs.velocityRotationsPerSecond = ABSOLUTE_ARM_ENCODER.getEncoderVelocity();

        armInputs.voltage = ARM_MOTOR.getVoltage();
    }
}
