package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.ShooterPhysicsCalculations;

import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.FLYWHEELS;
import static frc.robot.RobotContainer.INTAKE;
import static frc.robot.RobotContainer.KICKER;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;

public class ShooterCommands {
    public Command receiveFloorNote() {
        return ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
                .alongWith(
                        FLYWHEELS.setVoltage(-8),
                        INTAKE.setIntakeSpeed(0.5),
                        KICKER.setKickerPercentageOutput(-0.5)
                );
    }

    public Command outtakeNote() {
        return FLYWHEELS.setTargetVelocity(15)
                .alongWith(
                        INTAKE.setIntakeSpeed(-0.5),
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
//        final Trigger isReadyToShoot = new Trigger(() -> (FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget()));
        final Command waitAndShoot = new WaitCommand(2).andThen(KICKER.setKickerPercentageOutput(1));

        final ShooterPhysicsCalculations calculations = new ShooterPhysicsCalculations();

        final Command setArmPosition = ARM.setContinousTargetPosition(
                () -> calculations.getOptimalShootingAngleRadians(
                        POSE_ESTIMATOR.getCurrentPose(), target, tangentialVelocity
                ));

//        final ConditionalCommand shootKicker = new ConditionalCommand(
//                KICKER.setKickerPercentageOutput(1),
//                KICKER.setKickerPercentageOutput(0.0),
//                isReadyToShoot
//        );

        final Command setFlywheelVelocity = FLYWHEELS.setTargetTangentialVelocity(tangentialVelocity);

        return setArmPosition.alongWith(
                setFlywheelVelocity,
                waitAndShoot
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
