package frc.robot.subsystems.alternatearm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.motor.MotorInputsAutoLogged;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.util.commands.ExecuteEndCommand;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.alternatearm.NewArmConstants.ARM_MOTOR;
import static frc.robot.subsystems.alternatearm.NewArmConstants.SYSID_CONFIG;

public class NewArm extends SubsystemBase {
    private final SysIdRoutine routine;
    private MotorInputsAutoLogged armInputs = new MotorInputsAutoLogged();

    public NewArm() {
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (voltage) -> setRawVoltage(voltage.in(Volts)),
                this::sysIdLogArm,
                this
        );

        routine = new SysIdRoutine(SYSID_CONFIG, mechanism);
    }

    public boolean hasReachedTarget() {
        return ARM_MOTOR.isAtSetpoint();
    }

    public Command setTargetPosition(Rotation2d targetPosition) {
        return new ExecuteEndCommand(
                () -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,targetPosition.getRotations()),
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
        armInputs = ARM_MOTOR.getInputs();
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_MOTOR.setIdleMode(idleMode);
    }

    private void sysIdLogArm(SysIdRoutineLog log) {
        log.motor("Arm")
                .voltage(Volts.of(armInputs.voltage))
                .angularPosition(Rotations.of(armInputs.systemPosition))
                .angularVelocity(RotationsPerSecond.of(armInputs.systemVelocity));
    }

    private void setRawVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}
