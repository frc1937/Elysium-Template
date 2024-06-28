package frc.lib.util.mirrorable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;

/**
 * A class that allows for objects to be mirrored across the center of the field when the robot is on the red alliance.
 * This is useful for placing field elements and other objects that are mirrored across the field, or for mirroring the target heading to face a field element.
 *
 * @param <T> the type of object to mirror
 */
public abstract class Mirrorable<T> {
    protected final static Rotation2d ROTATION_180 = new Rotation2d(Math.PI);

    private final static Timer UPDATE_ALLIANCE_TIMER = new Timer();
    private final static Trigger UPDATE_ALLIANCE_TRIGGER = new Trigger(() -> UPDATE_ALLIANCE_TIMER.advanceIfElapsed(1.5));

    protected final T nonMirroredObject;
    protected final boolean shouldMirrorOnRed;

    protected static boolean cachedIsRedAlliance;

    static {
        UPDATE_ALLIANCE_TIMER.start();
        UPDATE_ALLIANCE_TRIGGER.onTrue(new InstantCommand(Mirrorable::updateAlliance));
    }

    /**
     * Creates a new mirrorable object.
     *
     * @param nonMirroredObject     the object when the robot is on the blue alliance, or the non-mirrored object
     * @param shouldMirrorOnRed whether to mirror the object when the robot is on the red alliance
     */
    protected Mirrorable(T nonMirroredObject, boolean shouldMirrorOnRed) {
        this.nonMirroredObject = nonMirroredObject;
        this.shouldMirrorOnRed = shouldMirrorOnRed;
    }

    /**
     * @return whether the robot is on the red alliance. This is cached every 0.5 seconds
     */
    public static boolean isRedAlliance() {
        return cachedIsRedAlliance;
    }

    /**
     * @return the current object.
     * If the robot is on the red alliance and the object should be mirrored, the mirrored object is returned.
     * Otherwise, the non-mirrored object is returned.
     */
    public T get() {
        return isRedAlliance() && shouldMirrorOnRed ? mirror(nonMirroredObject) : nonMirroredObject;
    }

    /**
     * Mirrors the object across the center of the field.
     *
     * @param object the object to mirror
     * @return the mirrored object
     */
    protected abstract T mirror(T object);


    private static void updateAlliance() {
        final Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();
        cachedIsRedAlliance = optionalAlliance.orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red);
    }
}
