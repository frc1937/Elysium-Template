// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Controller;
import frc.robot.commands.ShooterCommands;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.poseestimation.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(FRONT_CAMERA);
    public static final Swerve SWERVE = new Swerve();
    public static final Arm ARM = new Arm();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();

    private final ShooterCommands shooterCommands = new ShooterCommands();

    private final Controller driveController = new Controller(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("Teared down Crown"));

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        SWERVE.setDefaultCommand(SWERVE.driveTeleop(
                translationSupplier,
                strafeSupplier,
                () -> -driveController.getRawAxis(Controller.Axis.LEFT_STICK),
                driveController.getButton(Controller.Inputs.LEFT_BUMPER)
        ));

        driveController.getStick(Controller.Stick.RIGHT_STICK).whileTrue(shooterCommands.receiveFloorNote());

        ARM.setDefaultCommand(ARM.setTargetPosition(Rotation2d.fromDegrees(0)));

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

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
