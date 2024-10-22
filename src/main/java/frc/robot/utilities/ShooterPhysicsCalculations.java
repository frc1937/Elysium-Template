package frc.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.robot.GlobalConstants.GRAVITY_FORCE;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.LEFT_FLYWHEEL_DIAMETER;

public class ShooterPhysicsCalculations {
    private static TargetShootingState targetShootingState = new TargetShootingState(new Rotation2d(), new Rotation2d(), 0);


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
     * @return the robot's velocity relative to field in the xy plane
     */
    public static Translation3d getRobotFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = SWERVE.getFieldRelativeVelocity();
        return new Translation3d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond, 0);
    }

    /**
     * Predicts where the robot will be in a given amount of time on the xy plane.
     *
     * @param predictionTime the amount of time to predict the robot's position in, in seconds
     * @return the predicted position of the robot
     */
    private Translation2d predictFutureTranslation(double predictionTime) {
        final Translation2d fieldRelativeVelocity = getRobotFieldRelativeVelocity().toTranslation2d();
        final Translation2d currentPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();

        return currentPose.plus(fieldRelativeVelocity.times(predictionTime));
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     *
     * @param shootingTarget                             the point we want the note reach
     * @param standingShootingVelocityRotationsPerSecond the shooting velocity to calculate optimal pitch from, when the robot isn't moving.
     *                                                   This may change if the robot's velocity is not 0, but will act as a starting point
     * @param reachFromAbove                             should we reach to point from above, with an arch, or from below, as fast as possible.
     *                                                   Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShooterPhysicsCalculations.TargetShootingState}
     */
    private static TargetShootingState calculateTargetShootingState(Pose3d shootingTarget,
                                                                    double standingShootingVelocityRotationsPerSecond,
                                                                    boolean reachFromAbove) {

        final Translation2d currentTranslation = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Rotation2d standingTargetRobotAngle = getAngleFromPoseToPose(currentTranslation, shootingTarget.toPose2d().getTranslation());

        final double standingTangentialVelocity = Conversions.rpsToMps(standingShootingVelocityRotationsPerSecond, LEFT_FLYWHEEL_DIAMETER);

        final Rotation2d standingTargetPitch = calculateOptimalShootingAngle(standingTangentialVelocity,
                reachFromAbove,
                currentTranslation,
                standingTargetRobotAngle,
                shootingTarget);

        final TargetShootingState standingShootingState = new TargetShootingState(standingTargetRobotAngle, standingTargetPitch, standingTangentialVelocity);

        return calculateTargetShootingState(standingShootingState);
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     * This will calculate the TargetShootingState from a standing shooting state, to a dynamic "moving" shooting state to account for robot velocity.
     * The calculation is done using vector subtraction, where we subtract the robot's 3d vector from the note's 3d vector, and then find the new state from the end vector.
     * Note this actually uses vector addition since the shooter is placed in the opposite direction of the robot's velocity.
     *
     * @param standingShootingState the standing shooting state to calculate off of
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShooterPhysicsCalculations.TargetShootingState}
     */
    private static TargetShootingState calculateTargetShootingState(TargetShootingState standingShootingState) {
        final Translation3d noteVector = new Translation3d(standingShootingState.targetShootingVelocityRotationsPerSecond,
                new Rotation3d(0, -standingShootingState.targetPitch.getRadians(),
                        standingShootingState.targetRobotAngle.getRadians()));

        final Translation3d robotVector = getRobotFieldRelativeVelocity();
        final Translation3d shootingVector = noteVector.plus(robotVector);

        return calculateTargetShootingState(shootingVector);
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     * This will create the TargetShootingState from a shooting vector that was predetermined (most likely by a prior vector subtraction of the note and robot velocity vectors).
     *
     * @param shootingVector the predetermined shooting vector
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShooterPhysicsCalculations.TargetShootingState}
     */
    private static TargetShootingState calculateTargetShootingState(Translation3d shootingVector) {
        final Rotation2d targetRobotAngle = getYaw(shootingVector);
        final Rotation2d targetPitch = getPitch(shootingVector);



        final double targetVelocity = Conversions.mpsToRps(shootingVector.getNorm(), LEFT_FLYWHEEL_DIAMETER);

        return new TargetShootingState(targetRobotAngle, targetPitch, targetVelocity);
    }

    /**
     * Extracts the yaw off of a 3d vector.
     *
     * @param vector the vector to extract the yaw from
     * @return the yaw of the vector
     */
    private static Rotation2d getYaw(Translation3d vector) {
        return new Rotation2d(vector.getX(), vector.getY());
    }

    /**
     * Extracts the pitch off of a 3d vector.
     *
     * @param vector the vector to extract the pitch from
     * @return the pitch of the vector
     */
    private static Rotation2d getPitch(Translation3d vector) {
        return new Rotation2d(Math.atan2(vector.getZ(), Math.hypot(vector.getX(), vector.getY())));
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

        return calculatePitchAngleUsingPhysics(tangentialVelocity, noteExitPoseXYDistanceFromTarget, noteExitPoseHeightDifferenceFromTarget, reachFromAbove);
    }

    /**
     * Calculates the pitch the pitcher should reach in order to shoot at the shooting target using projectile motion.
     * This will fully calculate the target pitch using physics.
     *
     * @param noteTangentialVelocity                           the tangential velocity of the shooter
     * @param shooterNoteExitPointXYDistanceFromShootingTarget the xy distance from the shooting target to the shooter's note exit point on the xy plane
     * @param noteExitPointHeightDifferenceFromTarget          the height difference between the shooter's note exit point and the shooting target
     * @param reachFromAbove                                   should we reach to point from above, with an arch, or from below, as fast as possible
     *                                                         Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the pitch the robot should reach in order to shoot at the shooting target
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Projectile Motion</a>
     */
    private static Rotation2d calculatePitchAngleUsingPhysics(double noteTangentialVelocity, double shooterNoteExitPointXYDistanceFromShootingTarget, double noteExitPointHeightDifferenceFromTarget, boolean reachFromAbove) {
        Logger.recordOutput("ShootingCalculations/NoteTangentialVelocity", noteTangentialVelocity);
        Logger.recordOutput("ShootingCalculations/ShooterNoteExitPointHeight", noteExitPointHeightDifferenceFromTarget);

        final double gForce = GRAVITY_FORCE;
        final double velocitySquared = noteTangentialVelocity * noteTangentialVelocity;
        final double distanceSquared = shooterNoteExitPointXYDistanceFromShootingTarget * shooterNoteExitPointXYDistanceFromShootingTarget;

        final double squareRoot = Math.sqrt(
                velocitySquared * velocitySquared - (gForce * ((gForce * distanceSquared) + (2 * velocitySquared * noteExitPointHeightDifferenceFromTarget)))
        );

        final double numerator = reachFromAbove ? velocitySquared + squareRoot : velocitySquared - squareRoot;
        final double denominator = gForce * shooterNoteExitPointXYDistanceFromShootingTarget;
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

    public record TargetShootingState(Rotation2d targetRobotAngle, Rotation2d targetPitch,
                                      double targetShootingVelocityRotationsPerSecond) {
    }
}