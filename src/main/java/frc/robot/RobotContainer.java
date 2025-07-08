package frc.robot;

import static frc.robot.poseestimation.apriltagcamera.AprilTagCameraConstants.FRONT_LEFT_CAMERA;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.flippable.Flippable;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.PathPlannerConstants;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            FRONT_LEFT_CAMERA
//            FRONT_RIGHT_CAMERA,
//            REAR_LEFT_CAMERA,
//            REAR_RIGHT_CAMERA
    );

    public static final Swerve SWERVE = new Swerve();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
        PathPlannerConstants.initializePathPlanner();

        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TELEOP);
    }

    public Command getAutonomousCommand() {
        return new Command() {};
    }

    public String getAutoName() {
        return "None";
    }

    public void updateComponentPoses() {

    }
}