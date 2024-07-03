package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
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
        return Commands.run(() -> armIO.setTargetPosition(targetPosition), this);
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
