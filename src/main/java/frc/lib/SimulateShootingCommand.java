package frc.lib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static frc.robot.GlobalConstants.GRAVITY;
import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.FLYWHEELS;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class SimulateShootingCommand extends Command {
    private static final double X_DECELERATION = -1.5;

    private double initialVelocityMps;
    private double shootingAngleRotations;

    private Pose3d startingPose;
    private final Timer timer = new Timer();

    public SimulateShootingCommand() {
    }

    @Override
    public void initialize() {
        setupStartingValues();
    }

    @Override
    public void execute() {
        Logger.recordOutput("Note", calculateNewPose(startingPose, timer.get()));
    }

    private void setupStartingValues() {
        startingPose = new Pose3d(POSE_ESTIMATOR.getCurrentPose());
        initialVelocityMps = FLYWHEELS.getFlywheelTangentialVelocity();
        shootingAngleRotations = ARM.getCurrentAngleRotations();

        timer.restart();
    }

    private Pose3d calculateNewPose(Pose3d startingPose, double time) {
        final double initialVelocityX = initialVelocityMps * Math.cos(Units.rotationsToRadians(shootingAngleRotations));
        final double initialVelocityY = initialVelocityMps * Math.sin(Units.rotationsToRadians(shootingAngleRotations));

        final Transform3d transform = new Transform3d(
                initialVelocityX * time + X_DECELERATION * 1/2 * Math.pow(time, 2),
                0,
                initialVelocityY * time - GRAVITY * 1/2 * Math.pow(time, 2),
                new Rotation3d()
        );

        return startingPose.plus(transform);
    }
}
