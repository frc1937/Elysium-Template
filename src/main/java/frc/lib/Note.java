package frc.lib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.littletonrobotics.junction.Logger;

import java.util.UUID;

import static frc.robot.GlobalConstants.GRAVITY;

public class Note {
    private final double initialVelocity;
    private final Rotation2d yawAngle, pitchAngle;
    private final Pose3d startPose;

    private final Timer timer = new Timer();
    private final UUID id = UUID.randomUUID();

    public Note(Pose3d noteStartingPose, double initialVelocityMps, Rotation2d yawAngle, Rotation2d pitchAngle) {
        this.initialVelocity = initialVelocityMps;
        this.yawAngle = yawAngle;
        this.pitchAngle = pitchAngle;
        startPose = noteStartingPose;
    }

    /**
    Get the fly command for the note! Uses CMD to be periodically ran.
     */
    public Command fly() {
        return new FunctionalCommand(
                timer::start,
                () -> calculatePosition(timer.get()),
                interrupted -> {},
                () -> timer.advanceIfElapsed(5)
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
        final double yVelocity = diagonalVelocityCompensator * yawAngle.getSin();
        final double xVelocity = diagonalVelocityCompensator * yawAngle.getCos();

        final double timeSquared = time * time;
        final double frictionDeceleration = 1;

        return new Transform3d(new Translation3d(
                Math.max(xVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0),
                Math.max(yVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0),

                Math.max(zVelocity * time - 0.5 * GRAVITY * timeSquared, 0)),

                new Rotation3d());
    }
}
