package frc.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.robot.GlobalConstants.GRAVITY_FORCE;
import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.LEFT_FLYWHEEL_DIAMETER;

public class ShooterPhysicsCalculations {
    private static TargetShootingState targetShootingState = new TargetShootingState(new Rotation2d(), new Rotation2d());

    /**
     * Updates the {@linkplain ShooterPhysicsCalculations#targetShootingState} class variable to contain the target state for shooting at the speaker.
     */
    public static void updateCalculations(Pose3d shootingTarget, double flywheelVelocityRPS) {
        Logger.recordOutput("ShootingCalculations/TargetSpeakerPose", shootingTarget);
        targetShootingState = calculateTargetShootingState(shootingTarget, flywheelVelocityRPS, false);
    }

    /**
     * @return the target state of the robot to shoot at the provided shooting target
     */
    public static TargetShootingState getTargetShootingState() {
        return targetShootingState;
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     *
     * @param target                             the point we want the note reach
     * @param reachFromAbove                             should we reach to point from above, with an arch, or from below, as fast as possible.
     *                                                   Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShooterPhysicsCalculations.TargetShootingState}
     */
    private static TargetShootingState calculateTargetShootingState(Pose3d target,
                                                                    double flywheelVelocityRPS,
                                                                    boolean reachFromAbove) {

        final Translation2d currentTranslation = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Rotation2d standingTargetRobotAngle = getAngleFromPoseToPose(currentTranslation, target.toPose2d().getTranslation());

        final Rotation2d standingTargetPitch = calculateOptimalShootingAngle(
                Conversions.rpsToMps(flywheelVelocityRPS, LEFT_FLYWHEEL_DIAMETER),
                reachFromAbove,
                currentTranslation,
                standingTargetRobotAngle,
                target);

        return new TargetShootingState(standingTargetRobotAngle, standingTargetPitch);
    }

    /**
     * Calculates the optimal pitch for the given parameters, using the Projectile Motion calculation.
     *
     * @param tangentialVelocity the exit velocity of the note, as tangential velocity
     * @param reachFromAbove         should we reach to point from above, with an arch, or from below, as fast as possible
     *                               Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @param currentTranslation     the current translation of the robot, to find the note exit point's pose with
     * @param targetRobotAngle       the previously calculated target robot angle, to find the note exit point's pose with
     * @param target         the point we want the note reach
     * @return the pitch the pitcher should reach in order to shoot to the shooting target
     */
    private static Rotation2d calculateOptimalShootingAngle(double tangentialVelocity, boolean reachFromAbove, Translation2d currentTranslation,
                                                            Rotation2d targetRobotAngle, Pose3d target) {
        final Pose3d noteExitPose = getNoteExitPose(currentTranslation, targetRobotAngle);

        final double noteExitPoseXYDistanceFromTarget = noteExitPose.getTranslation().toTranslation2d().getDistance(target.toPose2d().getTranslation());
        final double noteExitPoseHeightDifferenceFromTarget = target.getZ() - noteExitPose.getZ();

        Logger.recordOutput("ShootingCalculations/NoteTangentialVelocity", tangentialVelocity);
        Logger.recordOutput("ShootingCalculations/ShooterNoteExitPointHeight", noteExitPoseHeightDifferenceFromTarget);

        final double gForce = GRAVITY_FORCE;
        final double velocitySquared = tangentialVelocity * tangentialVelocity;
        final double distanceSquared = noteExitPoseXYDistanceFromTarget * noteExitPoseXYDistanceFromTarget;

        final double squareRoot = Math.sqrt(
                velocitySquared * velocitySquared - (gForce * ((gForce * distanceSquared) + (2 * velocitySquared * noteExitPoseHeightDifferenceFromTarget)))
        );

        final double numerator = reachFromAbove ? velocitySquared + squareRoot : velocitySquared - squareRoot;
        final double denominator = gForce * noteExitPoseXYDistanceFromTarget;
        final double fraction = numerator / denominator;

        double angleRadians = Math.atan(fraction);

        if (Double.isNaN(angleRadians) || Double.isInfinite(angleRadians) || angleRadians < 0)
            angleRadians = Units.degreesToRadians(35);

        Logger.recordOutput("ShootingCalculations/TargetPitch", Math.toDegrees(angleRadians));
        return Rotation2d.fromRadians(angleRadians);
    }


    /**
     * Calculates the shooter's note exit point's 3d pose on the field from the given parameters.
     * The note exit point is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @param currentTranslation the field relative current translation, to base off from
     * @param robotAngle         the robot angle, to base off from
     * @return the shooter's note exit point's 3d pose on the field
     */
    public static Pose3d getNoteExitPose(Translation2d currentTranslation, Rotation2d robotAngle) {
        final double pitchTargetAngleRotations = ARM.getTargetAngleRotations();

        final Pose3d noteExitPoseSelfRelative = new Pose3d(PIVOT_POINT_X_OFFSET_METRES,0, PIVOT_POINT_Z_OFFSET_METRES, new Rotation3d())
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -Units.rotationsToRadians(pitchTargetAngleRotations), 0)))
                .plus(new Transform3d(SHOOTER_LENGTH_METRES, 0, 0, new Rotation3d()));

        Logger.recordOutput("ShootingCalculations/NoteExitPoint", noteExitPoseSelfRelative);
        //todo: calibrate maybe
        final Transform3d robotToNoteExitPose = noteExitPoseSelfRelative.minus(new Pose3d());
        final Pose3d currentPose = new Pose3d(new Pose2d(currentTranslation, robotAngle));

        return currentPose.transformBy(robotToNoteExitPose);
    }

    private static final double PIVOT_POINT_Z_OFFSET_METRES = 0.2;
    private static final double PIVOT_POINT_X_OFFSET_METRES = -0.31;
    private static final double SHOOTER_LENGTH_METRES = 0.415;

    public record TargetShootingState(Rotation2d targetRobotAngle, Rotation2d targetPitch) {
    }
}