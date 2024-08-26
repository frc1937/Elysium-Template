package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.littletonrobotics.junction.Logger;

import static frc.robot.GlobalConstants.GRAVITY;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class Note {
    private final double initialVelocity;
    private final Rotation2d pitchAngle;
    private final Rotation2d yawAngle;
    private Pose3d startPose;

    public Note(double initialVelocityMps, Rotation2d pitchAngle, Rotation2d yawAngle) {
        this.initialVelocity = initialVelocityMps;
        this.pitchAngle = pitchAngle;
        this.yawAngle = yawAngle;
    }

    public static Command createAndShootNote(double initialVelocity, Rotation2d pitchAngle) {
        return new FunctionalCommand(
                () -> {
                    final Note note = new Note(initialVelocity, pitchAngle, POSE_ESTIMATOR.getCurrentPose().getRotation());
                    note.fly().schedule();
                },
                () -> {},
                interrupted -> {},
                () -> false
        );
    }

    /**
    Get the fly command for the note! Uses CMD to be periodically ran.
     */
    private Command fly() {
        final Timer timer = new Timer();

        return new FunctionalCommand(
                () -> {
                    timer.start();
                    startPose = new Pose3d(POSE_ESTIMATOR.getCurrentPose());
                },
                () -> calculatePosition(timer.get()),
                interrupted -> {},
                () -> timer.advanceIfElapsed(7)
        );
    }

    public void calculatePosition(double time) {
        final Transform3d calculatedTranslation = getPhysicsPositionDelta(time);
        final Pose3d currentPose = startPose.transformBy(calculatedTranslation);

        Logger.recordOutput("Note", currentPose);
    }

    private Transform3d getPhysicsPositionDelta(double time) {
        final double zVelocity = initialVelocity * pitchAngle.getSin();

        final double diagonalVelocityCompensator = initialVelocity * pitchAngle.getCos();
        final double xVelocity = diagonalVelocityCompensator * yawAngle.getCos();
        final double yVelocity = diagonalVelocityCompensator * yawAngle.getSin();

        final double timeSquared = time * time;
        final double frictionDeceleration = 2.5;

        System.out.println("The y vel: " + Math.max(yVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0));
        System.out.println("X vel: " + Math.max(xVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0));

        return new Transform3d(new Translation3d(
                Math.max(xVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0),
                Math.max(yVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0),

                Math.max(zVelocity * time - 0.5 * GRAVITY * timeSquared, 0)),

                new Rotation3d());
    }
}
