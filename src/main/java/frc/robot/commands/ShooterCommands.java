package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {

    public Command receiveFloorNote() {
        return ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
                .alongWith(
                        FLYWHEEL.setFlywheelTarget(-15),
                        INTAKE.setIntakeSpeed(0.5),
                        KICKER.setKickerPercentageOutput(0.5) //TODO idk what direction.. needs testing
                );
    }
}
