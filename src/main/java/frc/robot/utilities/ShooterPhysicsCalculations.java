package frc.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

import static frc.robot.GlobalConstants.BLUE_SPEAKER;
import static frc.robot.GlobalConstants.GRAVITY_FORCE;
import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class ShooterPhysicsCalculations {
    public static final double PIVOT_POINT_Z_OFFSET_METRES = 0.2;
    private static final double PIVOT_POINT_X_OFFSET_METRES = -0.31;
    private static final double SHOOTER_LENGTH_METRES = 0.415;

    /**
     * @return the target angle of the robot to shoot at the provided target
     */
    public static Rotation2d calculateShootingAngle(Pose3d target, double flywheelVelocityRPS) {
        return Rotation2d.fromRadians(calculateOptimalShootingAngle(target, flywheelVelocityRPS, false));
    }

    public static Rotation2d zoneInOnShootingAngle(Pose3d target, double flywheelVelocityRPS) {
        double latestCalculatedAngleRotations = 0;

        System.out.println("NORMALLY CALCULATED ANGLE: " + calculateShootingAngle(target, flywheelVelocityRPS));

        for (int i = 0; i < 15; i++) {
            latestCalculatedAngleRotations = Units.radiansToRotations(calculateOptimalShootingAngle(
                    target, flywheelVelocityRPS, false, latestCalculatedAngleRotations));

            System.out.println("(" + i + ", " + latestCalculatedAngleRotations*360 + ")");
        }

        return Rotation2d.fromRotations(latestCalculatedAngleRotations);
    }

    public static Rotation2d zoneInOnShootingAngleGivenDistance(double xyDistance, Pose3d target, double flywheelVelocityRPS) {
        double latestCalculatedAngleRotations = 0;

//        System.out.println("NORMALLY CALCULATED ANGLE: " + calculateShootingAngle(target, flywheelVelocityRPS));

        for (int i = 0; i < 15; i++) {
            latestCalculatedAngleRotations = Units.radiansToRotations(
                    calculateAngleToPoseFromExitPoseTest(
                            xyDistance,
                    target, flywheelVelocityRPS, latestCalculatedAngleRotations)
            );

//            System.out.println("(" + i + ", " + latestCalculatedAngleRotations*360 + ")");
        }

        return Rotation2d.fromRotations(latestCalculatedAngleRotations);
    }

    /**
     * Calculates the optimal pitch for the given parameters, using Projectile Motion calculation.
     *
     * @param tangentialVelocity the exit velocity of the note, as tangential velocity
     * @param reachFromAbove     should we reach to point from above, with an arch, or from below, as fast as possible
     * @param target             the pose we want the note reach
     * @return the pitch to reach in order to shoot to the target
     */
    private static double calculateOptimalShootingAngle(Pose3d target, double tangentialVelocity, boolean reachFromAbove, double lastCalculatedAngleRotations) {
        final Pose3d noteExitPose = getNoteExitPose(lastCalculatedAngleRotations);

        return calculateAngleToPoseFromExitPose(target, tangentialVelocity, reachFromAbove, noteExitPose);
    }


    private static double calculateOptimalShootingAngle(Pose3d target, double tangentialVelocity, boolean reachFromAbove) {
        final Pose3d noteExitPose = getNoteExitPose();

        return calculateAngleToPoseFromExitPose(target, tangentialVelocity, reachFromAbove, noteExitPose);
    }

    private static double calculateAngleToPoseFromExitPose(Pose3d target, double tangentialVelocity, boolean reachFromAbove, Pose3d noteExitPose) {
        final double noteExitPoseXYDistanceFromTarget = noteExitPose.getTranslation().toTranslation2d().getDistance(target.toPose2d().getTranslation());
        final double noteExitPoseHeightDifferenceFromTarget = target.getZ() - noteExitPose.getZ();

        Logger.recordOutput("ShootingCalculations/NoteTangentialVelocity", tangentialVelocity);
        Logger.recordOutput("ShootingCalculations/ShooterNoteExitPointHeight", noteExitPoseHeightDifferenceFromTarget);

        final double velocitySquared = tangentialVelocity * tangentialVelocity;
        final double distanceSquared = noteExitPoseXYDistanceFromTarget * noteExitPoseXYDistanceFromTarget;

        final double squareRoot = Math.sqrt(
                velocitySquared * velocitySquared - (GRAVITY_FORCE * ((GRAVITY_FORCE * distanceSquared) + (2 * velocitySquared * noteExitPoseHeightDifferenceFromTarget)))
        );

        final double numerator = reachFromAbove ? velocitySquared + squareRoot : velocitySquared - squareRoot;
        final double denominator = GRAVITY_FORCE * noteExitPoseXYDistanceFromTarget;
        final double fraction = numerator / denominator;

        double angleRadians = Math.atan(fraction);

        if (Double.isNaN(angleRadians) || Double.isInfinite(angleRadians) || angleRadians < 0)
            angleRadians = Units.degreesToRadians(35);

        Logger.recordOutput("ShootingCalculations/TargetPitch", Math.toDegrees(angleRadians));
        Logger.recordOutput("ShootingCalculations/TargetPose", target);
        return angleRadians;
    }


    private static double calculateAngleToPoseFromExitPoseTest(double distanceXY, Pose3d target, double tangentialVelocity, double lastCalculatedAngleRotations) {
        final Pose3d noteExitPose = getNoteExitPoseForTesting(distanceXY, lastCalculatedAngleRotations);

        final double noteExitPoseXYDistanceFromTarget = noteExitPose.getTranslation().toTranslation2d().getDistance(target.toPose2d().getTranslation());
        final double noteExitPoseHeightDifferenceFromTarget = target.getZ() - noteExitPose.getZ();

        Logger.recordOutput("ShootingCalculations/NoteTangentialVelocity", tangentialVelocity);
        Logger.recordOutput("ShootingCalculations/ShooterNoteExitPointHeight", noteExitPoseHeightDifferenceFromTarget);

        final double velocitySquared = tangentialVelocity * tangentialVelocity;
        final double distanceSquared = noteExitPoseXYDistanceFromTarget * noteExitPoseXYDistanceFromTarget;

        final double squareRoot = Math.sqrt(
                velocitySquared * velocitySquared - (GRAVITY_FORCE * ((GRAVITY_FORCE * distanceSquared) + (2 * velocitySquared * noteExitPoseHeightDifferenceFromTarget)))
        );

        final double numerator = velocitySquared - squareRoot;
        final double denominator = GRAVITY_FORCE * noteExitPoseXYDistanceFromTarget;
        final double fraction = numerator / denominator;

        double angleRadians = Math.atan(fraction);

        if (Double.isNaN(angleRadians) || Double.isInfinite(angleRadians) || angleRadians < 0)
            angleRadians = Units.degreesToRadians(35);

        Logger.recordOutput("ShootingCalculations/TargetPitch", Math.toDegrees(angleRadians));
        Logger.recordOutput("ShootingCalculations/TargetPose", target);
        return angleRadians;
    }


    /**
     * Calculates the note's exit pose relative to the field
     * The note exit pose is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @return the shooter's note exit pose on the field
     */
    private static Pose3d getNoteExitPose(double currentTargetAngleRotations) {
        final Pose3d noteExitPoseSelfRelative = new Pose3d(PIVOT_POINT_X_OFFSET_METRES, 0, PIVOT_POINT_Z_OFFSET_METRES, new Rotation3d())
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -Units.rotationsToRadians(currentTargetAngleRotations), 0)))
                .plus(new Transform3d(SHOOTER_LENGTH_METRES, 0, 0, new Rotation3d()));

        Logger.recordOutput("ShootingCalculations/NoteExitPoint", noteExitPoseSelfRelative);

        final Transform3d robotToNoteExitPose = noteExitPoseSelfRelative.minus(new Pose3d());
        final Pose3d currentPose = new Pose3d(POSE_ESTIMATOR.getCurrentPose());

        return currentPose.transformBy(robotToNoteExitPose);
    }

    private static Pose3d getNoteExitPoseForTesting(double distanceXY, double currentTargetAngleRotations) {
        final Pose3d noteExitPoseSelfRelative = new Pose3d(PIVOT_POINT_X_OFFSET_METRES, 0, PIVOT_POINT_Z_OFFSET_METRES, new Rotation3d())
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -Units.rotationsToRadians(currentTargetAngleRotations), 0)))
                .plus(new Transform3d(SHOOTER_LENGTH_METRES, 0, 0, new Rotation3d()));

        final Transform3d robotToNoteExitPose = noteExitPoseSelfRelative.minus(new Pose3d());
        final Pose3d currentPose = new Pose3d(new Pose2d(new Translation2d
                (distanceXY, BLUE_SPEAKER.getY()), Rotation2d.fromDegrees(180)));

        return currentPose.transformBy(robotToNoteExitPose);
    }

    private static Pose3d getNoteExitPose() {
        return getNoteExitPose(ARM.getTargetAngleRotations());
    }
}