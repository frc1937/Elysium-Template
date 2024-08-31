package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.Note;
import frc.lib.math.Conversions;
import frc.robot.utilities.ShooterPhysicsCalculations;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {

    public Command receiveFloorNote() {
        return ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
                .alongWith(
                        FLYWHEELS.setTargetTangentialVelocity(-15),
                        INTAKE.setIntakeSpeed(0.5),
                        KICKER.setKickerPercentageOutput(0.5)
                );
    }

    public Command shootToTargetWithPhysics(Pose2d target, double tangentialVelocity) {
        return SWERVE.rotateToTarget(target).alongWith(
                FLYWHEELS.setTargetTangentialVelocity(tangentialVelocity),
                ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
        );
    }

    public Command shootPhysics(final Pose3d target, final double tangentialVelocity) {
        final ConditionalCommand shootFromKicker = new ConditionalCommand(
                KICKER.setKickerPercentageOutput(0.5),
                KICKER.setKickerPercentageOutput(0.0),
                () -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget()
        );

        final Command aimAtTargetPhysics = new ParallelCommandGroup(
                FLYWHEELS.setTargetTangentialVelocity(tangentialVelocity),
                ARM.setTargetPosition(Rotation2d.fromRadians(
                        new ShooterPhysicsCalculations()
                                .getOptimalShootingAngleRadians(POSE_ESTIMATOR.getCurrentPose(), target, tangentialVelocity))
                ));

        return new ParallelCommandGroup(
                aimAtTargetPhysics,
                shootFromKicker,
                Note.createAndShootNote(tangentialVelocity, Rotation2d.fromDegrees(ARM.getTargetAngleRotations()))
        );
    }

    public Command shootWithoutPhysics(double targetRPS, Rotation2d armAngle) {
        ConditionalCommand shootFromKicker = new ConditionalCommand(
                KICKER.setKickerPercentageOutput(0.5),
                KICKER.setKickerPercentageOutput(0.0),
                () -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget()
        );

        return new ParallelCommandGroup(
                ARM.setTargetPosition(armAngle),
                FLYWHEELS.setTargetVelocity(targetRPS),
                Note.createAndShootNote(Conversions.rpsToMps(targetRPS, Units.inchesToMeters(4)), armAngle),
                shootFromKicker
        );
    }
}
