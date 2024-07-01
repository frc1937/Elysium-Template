package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO armIO = ArmIO.generateArm();
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();

    public Command setTargetPosition(Rotation2d targetPosition) {
        return Commands.run(() -> armIO.setTargetPosition(targetPosition), this);
    }

    @Override
    public void periodic() {
        armIO.refreshInputs(armInputs);
        Logger.processInputs("Arm", armInputs);

        armIO.periodic();
    }
}
