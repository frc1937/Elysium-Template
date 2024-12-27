// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.Controller;
import frc.robot.commands.ShooterCommands;
import frc.robot.poseestimation.objectdetection.DetectionCameraFactory;
import frc.robot.poseestimation.objectdetection.DetectionCameraIO;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.GlobalConstants.BLUE_SPEAKER;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            FRONT_CAMERA
    );

    public static final DetectionCameraIO DETECTION_CAMERA = DetectionCameraFactory.createDetectionCamera("NotesCamera",
            new Transform3d(
                    new Translation3d(0.3, 0.08, 0.31),
                    new Rotation3d()
    ));

    public static final Swerve SWERVE = new Swerve();
    public static final Arm ARM = new Arm();
    public static final Flywheels FLYWHEELS = new Flywheels();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();
    public static final Leds LEDS = new Leds();

    public static final Trigger isNoteInShooter = new Trigger(KICKER::doesSeeNote);
    private final Trigger userButton = new Trigger(RobotController::getUserButton);

    private final Controller driveController = new Controller(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("Teared down Crown"));

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        setupLEDs();
        setupBrakeMode();

        configureButtons(ButtonLayout.TELEOP);
    }

    private void configureButtons(ButtonLayout layout) {
        switch (layout) {
            case CHARACTERIZE_ARM -> setupCharacterization(ARM);
            case CHARACTERIZE_FLYWHEEL -> setupCharacterization(FLYWHEELS);
            case TELEOP -> configureButtonsTeleop();
        }
    }

    private void setupCharacterization(GenericSubsystem subsystem) {
        driveController.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        driveController.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private enum ButtonLayout {
        TELEOP,
        CHARACTERIZE_FLYWHEEL,
        CHARACTERIZE_ARM
    }

    private void setupLEDs() {
        LEDS.setDefaultCommand(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0));

        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7) lowBatteryCounter[0]++;
            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        }).onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 5));

        isNoteInShooter
                .whileTrue(LEDS.setLEDStatus(Leds.LEDMode.SHOOTER_LOADED, 50))
                .toggleOnFalse(LEDS.setLEDStatus(Leds.LEDMode.SHOOTER_EMPTY, 3));
    }

    private void setupBrakeMode() {
        final Command setIdleModes = Commands.startEnd(
                () -> ARM.setIdleMode(MotorProperties.IdleMode.COAST),
                () -> ARM.setIdleMode(MotorProperties.IdleMode.BRAKE),
                ARM).ignoringDisable(true);

        final Command setLeds = LEDS.setLEDStatus(Leds.LEDMode.DEBUG_MODE, 15).ignoringDisable(true);

        userButton.toggleOnTrue(setIdleModes.alongWith(setLeds)).debounce(0.5);
    }

    private void setupDriving(DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier) {
        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,

                        () -> -driveController.getRawAxis(Controller.Axis.RIGHT_X) * 6,
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }

    private void configureButtonsTeleop() {
        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        setupDriving(translationSupplier, strafeSupplier);

//        driveController.getButton(Controller.Inputs.RIGHT_BUMPER)
//                .whileTrue(SwerveCommands.driveWhilstRotatingToTarget(translationSupplier, strafeSupplier,
//                                BLUE_SPEAKER.toPose2d(), () -> false)
//                        .alongWith(ShooterCommands.shootPhysics(BLUE_SPEAKER, 32))
//                );

//        driveController.getButton(Controller.Inputs.A)
//            .whileTrue(SwerveCommands.driveAndRotateToClosestNote(translationSupplier, strafeSupplier));

//        driveController.getButton(Controller.Inputs.B)
//                .whileTrue(ShooterCommands.shootPhysics(
//                        BLUE_SPEAKER, 32
//                ));

        driveController.getStick(Controller.Stick.RIGHT_STICK)
                .whileTrue(
                ShooterCommands.shootFromCalibrationTable(BLUE_SPEAKER)
        );

//        driveController.getButton(Controller.Inputs.B).whileTrue(
//                new WheelRadiusCharacterization(
//                        SWERVE,
//                        MODULE_LOCATIONS,
//                        SWERVE::getDriveWheelPositionsRadians,
//                        () -> SWERVE.getGyroHeading().getRadians(),
//                        SWERVE::runWheelCharacterization
//                ));

//        driveController.getButton(Controller.Inputs.X).whileTrue(
//                ShooterCommands.autoDetectAndShoot()
//        );
//
//        driveController.getButton(Controller.Inputs.Y).whileTrue(
//                ShooterCommands.calibrate()
//        );

        driveController.getButton(Controller.Inputs.A).whileTrue(
                ShooterCommands.shootWithoutPhysics(32, Rotation2d.fromDegrees(50)));

        driveController.getButton(Controller.Inputs.B).whileTrue(
                ShooterCommands.shootWithoutPhysics(10, Rotation2d.fromDegrees(90)));


        driveController.getButton(Controller.Inputs.X).whileTrue(
                ShooterCommands.shootWithoutPhysics(-30, Rotation2d.fromDegrees(-10)));


        driveController.getButton(Controller.Inputs.Y).whileTrue(
                ShooterCommands.shootWithoutPhysics(-15, Rotation2d.fromDegrees(43)));

        driveController.getStick(Controller.Stick.LEFT_STICK).whileTrue(ShooterCommands.receiveFloorNote());
        driveController.getButton(Controller.Inputs.LEFT_BUMPER).whileTrue(ShooterCommands.outtakeNote());
    }
}
