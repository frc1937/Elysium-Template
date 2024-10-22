package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.robot.utilities.ShooterPhysicsCalculations;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends GenericSubsystem {
    public Arm() {
        setName("Arm Subsystem");
        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    public boolean hasReachedTarget() {
        Logger.recordOutput("IsArmReady ", ARM_MOTOR.isAtPositionSetpoint());

        return ARM_MOTOR.isAtPositionSetpoint();
    }

    public double getVelocity() {
        return ARM_MOTOR.getSystemVelocity();
    }

    public void UNSAFE_setVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public Command setTargetPhysicsBasedPosition(Pose3d targetPose, double tangentialVelocity) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    Rotation2d targetAngle = ShooterPhysicsCalculations.calculateShootingAngle(targetPose, tangentialVelocity);
                    setMotorTargetPosition(targetAngle);
                },
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command setTargetPosition(Rotation2d targetPosition) {
        return new FunctionalCommand(
                () -> {},
                () -> setMotorTargetPosition(targetPosition),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    @Override
    public void periodic() {
        ARM_MECHANISM.updateMechanism(Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition()));
    }

    public double getTargetAngleRotations() {
        return ARM_MOTOR.getClosedLoopTarget();
    }

    public double getCurrentAngleRotations() {
        return ARM_MOTOR.getSystemPosition();
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_CONFIG;
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .voltage(Volts.of(ARM_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(ARM_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(ARM_MOTOR.getSystemVelocity()));
    }

    @Override
    public void sysIdDrive(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    private void setMotorTargetPosition(Rotation2d targetPosition) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,  targetPosition.getRotations());
        ARM_MECHANISM.setTargetAngle(targetPosition);
    }
}
