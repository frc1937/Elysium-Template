package frc.lib.util.mirrorable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.GlobalConstants;

public class MirrorablePose2d extends Mirrorable<Pose2d> {
    public MirrorablePose2d(Pose2d nonMirroredPose, boolean mirrorWhenRedAlliance) {
        super(nonMirroredPose, mirrorWhenRedAlliance);
    }

    public MirrorablePose2d(double x, double y, Rotation2d rotation, boolean mirrorWhenRedAlliance) {
        this(new Pose2d(x, y, rotation), mirrorWhenRedAlliance);
    }

    public MirrorablePose2d(Translation2d translation2d, double rotation, boolean mirrorWhenRedAlliance) {
        this(new Pose2d(translation2d, new Rotation2d(rotation)), mirrorWhenRedAlliance);
    }

    public MirrorableRotation2d getRotation() {
        return new MirrorableRotation2d(nonMirroredObject.getRotation(), shouldMirrorOnRed);
    }

    @Override
    protected Pose2d mirror(Pose2d pose) {
        return new Pose2d(
                GlobalConstants.FIELD_LENGTH_METRES - pose.getX(),
                pose.getY(),
                ROTATION_180.minus(pose.getRotation())
        );
    }
}
