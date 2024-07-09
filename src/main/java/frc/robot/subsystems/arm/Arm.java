package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.util.commands.ExecuteEndCommand;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.SYSID_CONFIG;

public class Arm extends SubsystemBase {
    private final ArmIO armIO = ArmIO.generateArm();
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();

    private final SysIdRoutine routine;

    public Arm() {
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (voltage) -> setRawVoltage(voltage.in(Volts)),
                this::sysIdLogArm,
                this
        );

        routine = new SysIdRoutine(SYSID_CONFIG, mechanism);
    }

    public boolean hasReachedTarget() {
        return armIO.hasReachedTarget();
    }

    public Command setTargetPosition(Rotation2d targetPosition) {
        return new ExecuteEndCommand(
                () -> armIO.setTargetPosition(targetPosition),
                armIO::stop,
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
        armIO.refreshInputs(armInputs);
        Logger.processInputs("Arm", armInputs);

        armIO.periodic();
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        armIO.setIdleMode(idleMode);
    }

    private void sysIdLogArm(SysIdRoutineLog log) {
        log.motor("Arm")
                .voltage(Volts.of(armInputs.voltage))
                .angularPosition(Rotations.of(armInputs.positionRotations))
                .angularVelocity(RotationsPerSecond.of(armInputs.velocityRotationsPerSecond));
    }

    private void setRawVoltage(double voltage) {
        armIO.setRawVoltage(voltage);
    }
}
