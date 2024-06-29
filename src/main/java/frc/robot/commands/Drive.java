package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

public class Drive extends Command {
    private final Swerve swerve;

    private final DoubleSupplier xSupplier, ySupplier, rotationSupplier;

    public Drive (Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble());
    }
}
