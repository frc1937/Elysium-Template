package frc.robot.subsystems.arm.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.arm.real.RealArmConstants.ABSOLUTE_ARM_ENCODER;
import static frc.robot.subsystems.arm.real.RealArmConstants.ARM_MOTOR;

public class RealArm extends ArmIO {
    private Rotation2d targetPosition = new Rotation2d();

    public RealArm() {
        resetRelativeEncoder();
    }

    private void resetRelativeEncoder() {
        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    @Override
    public void periodic() {
//        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
        //todo test without this
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        this.targetPosition = targetPosition;

        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }

    @Override
    public boolean hasReachedTarget() {
        Logger.recordOutput("IsArmAtSetpoint", ARM_MOTOR.isAtSetpoint());
        return ARM_MOTOR.isAtSetpoint();

//        if (targetPosition == null) return false;
//
//        return Math.abs(ABSOLUTE_ARM_ENCODER.getEncoderPosition() - targetPosition.getRotations()) < TOLERANCE_ROTATIONS;
    }

    @Override
    public void setRawVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    protected void stop() {
        ARM_MOTOR.stopMotor();
    }

    @Override
    protected void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public void refreshInputs(ArmInputsAutoLogged armInputs) {
        armInputs.positionRotations = ABSOLUTE_ARM_ENCODER.getEncoderPosition();
        armInputs.velocityRotationsPerSecond = ABSOLUTE_ARM_ENCODER.getEncoderVelocity();
        armInputs.targetRotations = targetPosition.getRotations();

        armInputs.voltage = ARM_MOTOR.getVoltage();
    }
}
