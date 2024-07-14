package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.util.commands.ExecuteEndCommand;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final SysIdRoutine routine;

    public Arm() {
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (voltage) -> setRawVoltage(voltage.in(Volts)),
                this::sysIdLogArm,
                this
        );

        routine = new SysIdRoutine(SYSID_CONFIG, mechanism);

        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    public boolean hasReachedTarget() {
        return ARM_MOTOR.isAtSetpoint();
    }

    public Command setTargetPosition(Rotation2d targetPosition) {
        return new ExecuteEndCommand(
                () -> setMotorTargetPosition(targetPosition),
                ARM_MOTOR::stopMotor,
                this);
    }

    public Command sysIdQuastaticTest(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamicTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (ARM_MOTOR.getClosedLoopTarget() != 0)
            ARM_MECHANISM.updateMechanism(
                    Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition()),
                    Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget())
            );
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_MOTOR.setIdleMode(idleMode);
    }

    private void sysIdLogArm(SysIdRoutineLog log) {
        log.motor("Arm")
                .voltage(Volts.of(ARM_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(ARM_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(ARM_MOTOR.getSystemVelocity()));
    }

    private void setRawVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    private void setMotorTargetPosition(Rotation2d targetPosition) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }
}
