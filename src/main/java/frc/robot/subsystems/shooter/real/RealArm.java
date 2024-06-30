package frc.robot.subsystems.shooter.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.shooter.ArmIO;

import static frc.robot.subsystems.shooter.real.RealArmConstants.ABSOLUTE_ARM_ENCODER;
import static frc.robot.subsystems.shooter.real.RealArmConstants.ARM_MOTOR;

public class RealArm extends ArmIO {

    @Override
    public void periodic() {
        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    @Override
    public void setTargetPosition(Rotation2d targetPosition) {
        ARM_MOTOR.setInput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }

    @Override
    public void refreshInputs(ArmInputsAutoLogged armInputs) {
        armInputs.positionRotations = ABSOLUTE_ARM_ENCODER.getEncoderPosition();
        armInputs.velocityRotationsPerSecond = ABSOLUTE_ARM_ENCODER.getEncoderVelocity();

        armInputs.voltage = ARM_MOTOR.getVoltage();
    }
}
