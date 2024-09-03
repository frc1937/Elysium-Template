package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.GlobalConstants.GRAVITY;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class Note {
    private double previousAngle = 0;

    private final DoubleSupplier initialVelocity;
    private Pose3d startPose;

    public Note(DoubleSupplier initialVelocityMps) {
        this.initialVelocity = initialVelocityMps;
    }

    public static Command createAndShootNote(DoubleSupplier initialVelocity, Supplier<Rotation2d> pitchAngle) {
         final Note note = new Note(initialVelocity);
         return note.fly(pitchAngle);
    }

    /**
    Get the fly command for the note! Uses CMD to be periodically ran.
     */
    private Command fly(Supplier<Rotation2d> pitchAngle) {
        final Timer timer = new Timer();
        final Rotation2d[] currentPitchAngle = {pitchAngle.get()};
        final double[] currentVelocity = {initialVelocity.getAsDouble()};

        return new FunctionalCommand(
                () -> {
                    startPose = new Pose3d(POSE_ESTIMATOR.getCurrentPose());
                    timer.restart();

                    currentPitchAngle[0] = pitchAngle.get();
                    currentVelocity[0] = initialVelocity.getAsDouble();
                },
                () -> calculatePosition(timer.get(), currentPitchAngle[0], currentVelocity[0]),

                interrupted -> timer.reset(),
                () -> timer.hasElapsed(5)
        );
    }

    private void calculatePosition(double time, Rotation2d pitchAngle, double initialVelocity) {
        final Transform3d calculatedTranslation = getPhysicsPositionDelta(time, pitchAngle, initialVelocity);
        final Pose3d currentPose = startPose.transformBy(calculatedTranslation);

        Logger.recordOutput("Note", currentPose);
    }

    private Transform3d getPhysicsPositionDelta(double time, Rotation2d pitchAngle, double initialVelocity) {
        final double zVelocity = initialVelocity * pitchAngle.getSin();
        final double xyVelocity = initialVelocity * pitchAngle.getCos();

        final double timeSquared = time * time;
        final double frictionDeceleration = 1.5;

        if (previousAngle != pitchAngle.getDegrees()) {
            System.out.println("Release speed " + initialVelocity + " m/s" + ", Release angle: " + pitchAngle.getDegrees() + " degrees");
            previousAngle = pitchAngle.getDegrees();
        }

        return new Transform3d(new Translation3d(
                Math.max(xyVelocity * time - 0.5 * frictionDeceleration * timeSquared, 0),
                0,
                Math.max(zVelocity * time - 0.5 * GRAVITY * timeSquared, 0)),
                new Rotation3d());
    }
}
