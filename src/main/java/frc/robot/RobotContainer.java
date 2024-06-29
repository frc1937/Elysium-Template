// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.Controller;
import frc.robot.commands.Drive;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Controller controller = new Controller(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        controller.getStick(Controller.Stick.LEFT_STICK).onTrue(new InstantCommand(() -> swerve.drive(0.5, 0, 0)));

        swerve.setDefaultCommand(new Drive(
                swerve,
                () -> {
                    System.out.println("Value: " + -controller.getRawAxis(Controller.Axis.LEFT_Y));
                    return -controller.getRawAxis(Controller.Axis.LEFT_Y);
                },
                () -> -controller.getRawAxis(Controller.Axis.LEFT_X),
                () -> -controller.getRawAxis(Controller.Axis.RIGHT_X)
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
