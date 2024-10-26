// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import org.littletonrobotics.junction.LoggedRobot;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.poseestimation.photoncamera.CameraFactory.VISION_SIMULATION;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        HardwareManager.initialize(this);
    }

    @Override
    public void robotPeriodic() {
        commandScheduler.run();
        HardwareManager.update();

        SWERVE.periodicallyUpdateFromOdometry();
        POSE_ESTIMATOR.periodic();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();

        SWERVE.periodicallyUpdateFromOdometry();
        VISION_SIMULATION.updateRobotPose(POSE_ESTIMATOR.getOdometryPose());

        final Field2d simulatedVisionField = VISION_SIMULATION.getDebugField();

        simulatedVisionField.getObject("EstimatedRobot").setPose(POSE_ESTIMATOR.getCurrentPose());
    }

    @Override
    public void close() {
        super.close();
    }
}
