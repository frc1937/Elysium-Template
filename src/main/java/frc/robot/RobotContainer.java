// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.Controller;
import frc.robot.commands.ShooterCommands;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();
    public static final Swerve SWERVE = new Swerve();
    public static final Arm ARM = new Arm();
    public static final Flywheels FLYWHEEL = new Flywheels();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();
    public static final Leds LEDS = new Leds();

    private final ShooterCommands shooterCommands = new ShooterCommands();

    private final Controller driveController = new Controller(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("Teared down Crown"));

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        LEDS.setDefaultCommand(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0));
        new Trigger(() -> RobotController.getBatteryVoltage() < 12).onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 10));

        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        SWERVE.setDefaultCommand(
                SWERVE.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,
                        () -> -driveController.getRawAxis(Controller.Axis.RIGHT_X),
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SWERVE.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SWERVE.lockSwerve());

        driveController.getButton(Controller.Inputs.A)
                .whileTrue(shooterCommands.shootWithoutPhysics(25, Rotation2d.fromDegrees(35)));

        configureButtons(ButtonLayout.CHARACTERIZE_ARM);

//
//        driveController.getStick(Controller.Stick.RIGHT_STICK).whileTrue(shooterCommands.receiveFloorNote());
//
//        FLYWHEEL.sysIdDynamicTest(SysIdRoutine.Direction.kForward);

//        new Trigger(driveController.getButton(Controller.Inputs.B)).whileTrue(
//                shooterCommands.shootToTarget(BLUE_SPEAKER.toPose2d(), 15)
//        );

//        driveController.getButton(Controller.Inputs.BACK).onTrue(SWERVE.resetGyro());

//        driveController.getButton(Controller.Inputs.A).whileTrue(
//                SWERVE.driveWhilstRotatingToTarget(
//                        translationSupplier, strafeSupplier, BLUE_SPEAKER.toPose2d(),
//                        driveController.getButton(Controller.Inputs.LEFT_BUMPER)
//                ));
    }

    private void configureButtons(ButtonLayout layout) {
        switch (layout) {
            case CHARACTERIZE_ARM -> {
                driveController.getButton(Controller.Inputs.A).whileTrue(ARM.sysIdDynamicTest(SysIdRoutine.Direction.kForward));
                driveController.getButton(Controller.Inputs.B).whileTrue(ARM.sysIdDynamicTest(SysIdRoutine.Direction.kReverse));
                driveController.getButton(Controller.Inputs.Y).whileTrue(ARM.sysIdQuastaticTest(SysIdRoutine.Direction.kForward));
                driveController.getButton(Controller.Inputs.X).whileTrue(ARM.sysIdQuastaticTest(SysIdRoutine.Direction.kReverse));
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private enum ButtonLayout {
        TELEOP,
        CHARACTERIZE_FLYWHEEL,
        CHARACTERIZE_ARM
    }
}
