// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Controller;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
    public static final Swerve SWERVE = new Swerve();

    private final Controller controller = new Controller(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        SWERVE.setDefaultCommand(SWERVE.drive(
                () -> -controller.getRawAxis(Controller.Axis.LEFT_X),
                () -> controller.getRawAxis(Controller.Axis.LEFT_Y),
                () -> 0//controller.getRawAxis(Controller.Axis.LEFT_X)
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
