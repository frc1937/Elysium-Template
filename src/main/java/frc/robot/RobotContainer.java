// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Controller;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.*;
import static frc.robot.GlobalConstants.BLUE_SPEAKER;

public class RobotContainer {
    public static final Swerve SWERVE = new Swerve();
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();

    private final Controller driveController = new Controller(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        SWERVE.setDefaultCommand(SWERVE.driveTeleop(
                translationSupplier,
                strafeSupplier,
                () -> -driveController.getRawAxis(RIGHT_X) //todo: figure out how to do rot in sim lol
        ));

        driveController.getButton(Controller.Inputs.Y).onTrue(SWERVE.resetGyro());

        driveController.getButton(Controller.Inputs.A).whileTrue(SWERVE.driveWhilstRotatingToTarget(
                translationSupplier, strafeSupplier, BLUE_SPEAKER.toPose2d()
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
