package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SimulateShootingCommand;
import frc.robot.subsystems.swerve.SwerveCommands;

import static frc.robot.GlobalConstants.BLUE_SPEAKER;
import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.DETECTION_CAMERA;
import static frc.robot.RobotContainer.FLYWHEELS;
import static frc.robot.RobotContainer.INTAKE;
import static frc.robot.RobotContainer.KICKER;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.isNoteInShooter;
import static frc.robot.subsystems.arm.ArmConstants.CURRENT_ANGLE_CALIB;
import static frc.robot.subsystems.arm.ArmConstants.SHOOTING_ANGLES;

public class ShooterCommands {
    public static Command autoDetectAndShoot() {
        final Trigger isNoteVisible = new Trigger(DETECTION_CAMERA::hasResult);

        final Command keepOnRotating = SwerveCommands.driveOpenLoop(() -> 0, () -> 0, () -> 1.8, () -> true);
        final Command gotoToClosestNote =
                SwerveCommands.driveToClosestNote()
                .until(isNoteInShooter);

        final Command intakeFloorNote = receiveFloorNote().until(isNoteInShooter)
                .andThen(receiveFloorNote().withTimeout(0.5));

        final Command rotateToSpeakerAndShoot = SwerveCommands.rotateToTarget(BLUE_SPEAKER.toPose2d())
                .alongWith(shootPhysics(BLUE_SPEAKER, 32));

        final Command pickupNoteAndShoot = gotoToClosestNote.alongWith(intakeFloorNote)
                .andThen(rotateToSpeakerAndShoot);

        return new ConditionalCommand(
                pickupNoteAndShoot,
                keepOnRotating,
//                        .until(isNoteVisible).andThen(pickupNoteAndShoot),

                isNoteVisible
        );
    }

    public static Command calibrate() {
        final Command timer = new WaitCommand(2);

        return FLYWHEELS.setTargetTangentialVelocityFromRPS(32).alongWith(
                ARM.setTargetPositionSupplier(CURRENT_ANGLE_CALIB),
                timer,
//f\left(x\right)=\frac{180}{\pi}\tan\left(\frac{1.88}{x}\right)\left\{0<x\right\}
                KICKER.setKickerPercentageOutput(0)
                        .until(() -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget() ||
                                timer.isFinished())

                        .andThen(KICKER.setKickerPercentageOutput(0.7).alongWith(simulateNoteShooting()).alongWith(
                                new InstantCommand(() ->
                                System.out.println("\n-----------------\n" +
                                        "TO PUT ANGLE DISTANCE AHHH " +
                                        "\n(" + POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(BLUE_SPEAKER.toPose2d().getTranslation())
                                        + ", "+ CURRENT_ANGLE_CALIB.getAsDouble() + ")" +
                                        "\n---------------------"
                                ))
                        ))
        );
    }

    public static Command receiveFloorNote() {
        return ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
                .alongWith(
                        FLYWHEELS.setVoltage(-4.5),
                        INTAKE.setIntakeSpeedPercentage(1),
                        KICKER.setKickerPercentageOutput(-0.6)
                );
    }

    public static Command outtakeNote() {
        return FLYWHEELS.setTargetVelocity(15)
                .alongWith(
                        INTAKE.setIntakeSpeedPercentage(-0.5),
                        KICKER.setKickerPercentageOutput(0.5)
                );
    }

    public static Command shootToTargetWithPhysics(Pose2d target, double tangentialVelocity) {
        return SwerveCommands.rotateToTarget(target).alongWith(
                FLYWHEELS.setTargetTangentialVelocity(tangentialVelocity),
                ARM.setTargetPosition(Rotation2d.fromDegrees(-20))
        );
    }

    public static Command shootFromCalibrationTable(Pose3d target) {
        final Command setFlywheelVelocity = FLYWHEELS.setTargetTangentialVelocityFromRPS(32);
        final Command setArmPosition = ARM.setTargetPositionSupplier(() ->
                SHOOTING_ANGLES.get(POSE_ESTIMATOR
                        .getCurrentPose().getTranslation().getDistance(target.toPose2d().getTranslation())));

        final Command timer = new WaitCommand(2);

        return setFlywheelVelocity.alongWith(
                setArmPosition,
                timer,
                KICKER.setKickerPercentageOutput(0)
                        .until(() -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget() ||
                                timer.isFinished())

                        .andThen(KICKER.setKickerPercentageOutput(0.7).alongWith(simulateNoteShooting()))
        );
    }

    public static Command shootPhysics(Pose3d target, double targetVelocityRPS) {
        final Command setFlywheelVelocity = FLYWHEELS.setTargetTangentialVelocityFromRPS(targetVelocityRPS);
        final Command setArmPosition = ARM.setTargetPhysicsBasedPosition(target, targetVelocityRPS);
        final Command timer = new WaitCommand(2);

        return setFlywheelVelocity.alongWith(
                setArmPosition,
                timer,
                KICKER.setKickerPercentageOutput(0)
                        .until(() -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget() ||
                                timer.isFinished())

                        .andThen(KICKER.setKickerPercentageOutput(0.7).alongWith(simulateNoteShooting()))
        );
    }

    public static Command shootPhysicsNoZoneIn(Pose3d target, double targetVelocityRPS) {
        final Command setFlywheelVelocity = FLYWHEELS.setTargetTangentialVelocityFromRPS(targetVelocityRPS);
        final Command setArmPosition = ARM.setTargetPhysicsBasedPositionNoZoneIn(target, targetVelocityRPS);
        final Command timer = new WaitCommand(4);

        return setFlywheelVelocity.alongWith(
                setArmPosition,
                timer,
                KICKER.setKickerPercentageOutput(0)
                        .until(() -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget() ||
                                timer.isFinished())

                        .andThen(KICKER.setKickerPercentageOutput(0.7).alongWith(simulateNoteShooting()))
        );
    }

    public static Command shootWithoutPhysics(double targetRPS, Rotation2d armAngle) {
        ConditionalCommand shootFromKicker = new ConditionalCommand(
                KICKER.setKickerPercentageOutput(0.5),
                KICKER.setKickerPercentageOutput(0.0),
                () -> FLYWHEELS.hasReachedTarget() && ARM.hasReachedTarget()
        );

        return new ParallelCommandGroup(
                ARM.setTargetPosition(armAngle),
                FLYWHEELS.setTargetVelocity(targetRPS),
                simulateNoteShooting(),
                shootFromKicker
        );
    }

    public static Command simulateNoteShooting() {
        return new InstantCommand(
                () -> new SimulateShootingCommand()
                        .schedule());
    }
}
