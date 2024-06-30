package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final ArmIO armIO = ArmIO.generateArm();

    public Command setTargetPosition(Rotation2d targetPosition) {
        return Commands.run(() -> armIO.setTargetPosition(targetPosition), this);
    }

    @Override
    public void periodic() {
        armIO.periodic();
    }
}
