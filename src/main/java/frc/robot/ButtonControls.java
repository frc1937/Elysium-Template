package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.flippable.Flippable;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveCommands.rotateToTarget;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class ButtonControls {
        public enum ButtonLayout {
                TELEOP,
                CHARACTERIZE_SWERVE_DRIVE_MOTORS,
                CHARACTERIZE_WHEEL_RADIUS,
                CHARACTERIZE_SWERVE_AZIMUTH,
        }

        private static final Controller DRIVER_CONTROLLER = new Controller(0);

        public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

        private static final DoubleSupplier X_SUPPLIER = () -> DRIVE_SIGN.getAsDouble()
                        * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
        private static final DoubleSupplier Y_SUPPLIER = () -> DRIVE_SIGN.getAsDouble()
                        * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
        private static final DoubleSupplier ROTATION_SUPPLIER = () -> -DRIVER_CONTROLLER
                        .getRawAxis(Controller.Axis.RIGHT_X) * 8;

        private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

        public static void initializeButtons(ButtonLayout layout) {
                setupUserButtonDebugging();

                switch (layout) {
                        case TELEOP -> configureButtonsTeleop();
                        case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
                        case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> {
                                setupDriving();
                                setupSysIdCharacterization(SWERVE);
                        }
                        case CHARACTERIZE_SWERVE_AZIMUTH -> setupAzimuthCharacterization();
                }
        }

        private static void configureButtonsTeleop() {
                setupDriving();
        }

        private static void configureButtonsCharacterizeWheelRadius() {
                setupDriving();

                final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                                SWERVE,
                                ROBOT_CONFIG.moduleLocations,
                                SWERVE::getDriveWheelPositionsRadians,
                                () -> SWERVE.getGyroHeading() * 2 * Math.PI,
                                (speed) -> SWERVE
                                                .driveRobotRelative(new ChassisSpeeds(0, 0, speed), true));

                DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue((wheelRadiusCharacterization));
        }

        private static void setupSysIdCharacterization(GenericSubsystem subsystem) {
                DRIVER_CONTROLLER.getButton(Controller.Inputs.A)
                                .whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
                DRIVER_CONTROLLER.getButton(Controller.Inputs.B)
                                .whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
                DRIVER_CONTROLLER.getButton(Controller.Inputs.Y)
                                .whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
                DRIVER_CONTROLLER.getButton(Controller.Inputs.X)
                                .whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        private static void setupUserButtonDebugging() {
                USER_BUTTON.toggleOnTrue(
                                Commands.startEnd(
                                                () -> setModeOfAllSubsystems(MotorProperties.IdleMode.COAST),
                                                () -> setModeOfAllSubsystems(MotorProperties.IdleMode.BRAKE)))
                                .debounce(1);
        }

        private static void setModeOfAllSubsystems(MotorProperties.IdleMode idleMode) {
        }

        private static void setupAzimuthCharacterization() {
                DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(
                                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(90))));

                DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(
                                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(180))));

                DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(
                                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(270))));

                DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(
                                rotateToTarget(POSE_ESTIMATOR.getCurrentPose().rotateBy(Rotation2d.fromDegrees(360))));
        }

        private static void setupDriving() {
                SWERVE.setDefaultCommand(
                                SwerveCommands.driveOpenLoop(
                                                X_SUPPLIER,
                                                Y_SUPPLIER,
                                                ROTATION_SUPPLIER,

                                                () -> false));

                DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
                DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
        }
}
