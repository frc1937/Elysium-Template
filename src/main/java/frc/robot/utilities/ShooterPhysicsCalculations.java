package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.GlobalConstants.GRAVITY_FORCE;
import static frc.robot.RobotContainer.ARM;

public class ShooterPhysicsCalculations {
    private static final double PIVOT_POINT_Z_OFFSET_METRES = 0.21;
    private static final double PIVOT_POINT_X_OFFSET_METRES = -0.31;
    private static final double SHOOTER_LENGTH_METRES = 0.485;

    public double getOptimalShootingAngleRadians(final Pose2d robotPose, final Pose3d targetPose, final double tangentialVelocity) {
        final double optimalAngleNoMovement = getPhysicsShootingAngleRadians(robotPose, targetPose, tangentialVelocity);

//        final Pose3d targetFromMovingRobot = getTargetFromMovingRobot(targetPose, optimalAngleNoMovement,
//                getDistanceToTarget(robotPose, targetPose), tangentialVelocity, SWERVE.getSelfRelativeVelocity());
//todo: SOTM
        return optimalAngleNoMovement;// getPhysicsShootingAngleRadians(robotPose, targetFromMovingRobot, tangentialVelocity);
    }

    private double getPhysicsShootingAngleRadians(final Pose2d robotPose, final Pose3d targetPose, final double tangentialVelocity) {
        final Pose3d exitPose = getNoteExitPose(robotPose, targetPose);

        final double vSquared = tangentialVelocity * tangentialVelocity;
        final double vQuad = vSquared * vSquared;
        final double z = targetPose.getZ() - exitPose.getZ();
        final double distance = getDistanceToTarget(exitPose, targetPose);

        final double discriminant = vQuad - GRAVITY_FORCE * (GRAVITY_FORCE * distance * distance + 2 * vSquared * z);

        if (discriminant < 0) {
            throw new IllegalArgumentException("No valid shooting angle: discriminant is negative");
        }

        final double sqrt = Math.sqrt(discriminant);
        final double denominator = GRAVITY_FORCE * distance;

        double theta = Math.atan((vSquared - sqrt) / denominator);

        if (Double.isNaN(theta) || Double.isInfinite(theta) || theta < 0) {
            //Use the other solution if the angle is invalid
            System.out.println("Using other angle, first is invalid");

            theta = Math.atan((vSquared + sqrt) / denominator);
        }

        SmartDashboard.putNumber("physics/DivNumerator", (vSquared + sqrt));
        SmartDashboard.putNumber("physics/DivDenominator", GRAVITY_FORCE * distance);
        SmartDashboard.putNumber("physics/DivResult", (vSquared + sqrt) / (GRAVITY_FORCE * distance));
        SmartDashboard.putNumber("physics/ZedDistance", z);
        SmartDashboard.putNumber("physics/distance", distance);
        SmartDashboard.putString("physics/robotPose", exitPose.toString());
        SmartDashboard.putNumber("physics/FunctionTheta Deg", Units.radiansToDegrees(theta));


        return theta;
    }

    private Pose3d getTargetFromMovingRobot(final Pose3d targetPose,
                                                   final double shootingAngleRadians,
                                                   final double distanceToTarget,
                                                   final double tangentialVelocity,
                                                   final ChassisSpeeds robotVelocity) {
        final double timeOfFlight = getTimeOfFlight(shootingAngleRadians, distanceToTarget, tangentialVelocity);

        final Transform3d targetOffset = new Transform3d(
                robotVelocity.vxMetersPerSecond * timeOfFlight,
                robotVelocity.vyMetersPerSecond * timeOfFlight,
                0,
                new Rotation3d()
        );

        return targetPose.transformBy(targetOffset.inverse());
    }

    private double getTimeOfFlight(final double shootingAngleRadians, final double distanceToTarget, final double tangentialVelocity) {
        final double speed = tangentialVelocity * Math.cos(shootingAngleRadians);

        return distanceToTarget / speed;
    }

    private double getDistanceToTarget(final Pose2d robotPose, final Pose3d target) {
        return getDistanceToTarget(new Pose3d(robotPose), target);
    }

    private double getDistanceToTarget(final Pose3d robotPose, final Pose3d target) {
        return robotPose.getTranslation().getDistance(target.getTranslation());
    }

    private Pose3d getNoteExitPose(Pose2d robotPose, Pose3d targetPose) {
        final Pose3d robotPose3d = new Pose3d(new Pose2d(robotPose.getTranslation(), targetPose.getRotation().toRotation2d()));

        final Transform3d robotToPivot = new Transform3d(
                PIVOT_POINT_X_OFFSET_METRES, 0, PIVOT_POINT_Z_OFFSET_METRES,
                new Rotation3d(0, -ARM.getTargetAngleRotations() * 2 * Math.PI, 0)
        );

        final Transform3d pivotToShooterEnd = new Transform3d(SHOOTER_LENGTH_METRES, 0, 0, new Rotation3d());

        final Pose3d shooterEndPose = new Pose3d().transformBy(robotToPivot).plus(pivotToShooterEnd);
        final Transform3d robotToShooterEnd = shooterEndPose.minus(new Pose3d());

        return robotPose3d.transformBy(robotToShooterEnd);
    }
}
