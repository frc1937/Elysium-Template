// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Controller;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.subsystems.shooter.Arm;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.*;
import static frc.robot.GlobalConstants.BLUE_SPEAKER;
import static frc.robot.poseestimation.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(FRONT_CAMERA);
    public static final Swerve SWERVE = new Swerve();
    public static final Arm ARM = new Arm();

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
                () -> -driveController.getRawAxis(RIGHT_Y),
                driveController.getButton(Controller.Inputs.LEFT_BUMPER)
        ));

        driveController.getDPad(Controller.DPad.DOWN)
                .whileTrue(ARM.setTargetPosition(Rotation2d.fromDegrees(50)));

        ARM.setDefaultCommand(ARM.setTargetPosition(Rotation2d.fromDegrees(0)));

        driveController.getButton(Controller.Inputs.BACK).onTrue(SWERVE.resetGyro());

        driveController.getButton(Controller.Inputs.A).whileTrue(
                SWERVE.driveWhilstRotatingToTarget(
                        translationSupplier, strafeSupplier, BLUE_SPEAKER.toPose2d(),
                        driveController.getButton(Controller.Inputs.LEFT_BUMPER)
                ));

        Logger.recordOutput("SpeakerPOS", BLUE_SPEAKER);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
