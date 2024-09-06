package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Note;
import frc.robot.utilities.ShooterPhysicsCalculations;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {

    public Command receiveFloorNote() {
        return ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
                .alongWith(
                        FLYWHEELS.setTargetTangentialVelocity(-5),
                        INTAKE.setIntakeSpeed(0.5),
                        KICKER.setKickerPercentageOutput(-0.5)
                );
    }

    public Command shootToTargetWithPhysics(Pose2d target, double tangentialVelocity) {
        return SWERVE.rotateToTarget(target).alongWith(
                FLYWHEELS.setTargetTangentialVelocity(tangentialVelocity),
                ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
        );
    }

    public Command shootPhysics(final Pose3d target, final double tangentialVelocity) {
        final Trigger isShooterAtDesiredState = new Trigger(() -> {
            System.out.println("Flywheels: " + FLYWHEELS.hasReachedTarget() + " Arm: " + ARM.hasReachedTarget());

            return FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget();
        });

        final ConditionalCommand shootFromKicker = new ConditionalCommand(
                KICKER.setKickerPercentageOutput(0.5).alongWith(
                        Note.createAndShootNote(FLYWHEELS::getFlywheelTangentialVelocity,
                                () -> Rotation2d.fromRotations(ARM.getCurrentAngleRotations())
                        )
                ),
                KICKER.setKickerPercentageOutput(0.0),
                isShooterAtDesiredState
        );

        final Command aimAtTargetPhysics = new ParallelCommandGroup(
                FLYWHEELS.setTargetTangentialVelocity(tangentialVelocity),
                ARM.setContinuousTargetPosition(() -> Rotation2d.fromRadians(
                        new ShooterPhysicsCalculations()
                                .getOptimalShootingAngleRadians(POSE_ESTIMATOR.getCurrentPose(), target, tangentialVelocity))
                ));

        return new ParallelCommandGroup(
                aimAtTargetPhysics,
                shootFromKicker
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
//                Note.createAndShootNote(Conversions.rpsToMps(targetRPS, Units.inchesToMeters(4)),
//                        () -> Rotation2d.fromRotations(ARM.getTargetAngleRotations())),
                shootFromKicker
        );
    }
}
