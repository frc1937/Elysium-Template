package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.OdometryThread;
import frc.lib.math.Optimizations;
import frc.lib.util.commands.InitExecuteCommand;
import frc.lib.util.mirrorable.Mirrorable;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.lib.math.Conversions.proportionalPowerToMps;
import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;

public class Swerve extends GenericSubsystem {
    private double lastTimestamp = Timer.getFPGATimestamp();

    public Swerve() {
        configurePathPlanner();
    }

    public Command lockSwerve() {
        return Commands.run(
                () -> {
                    final SwerveModuleState
                            right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                            left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

                    MODULES[0].setTargetState(left);
                    MODULES[1].setTargetState(right);
                    MODULES[2].setTargetState(right);
                    MODULES[3].setTargetState(left);
                },
                this
        );
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> this.setGyroHeading(Rotation2d.fromDegrees(0)), this);
    }

    public Command driveOpenLoop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier robotCentric) {
        return new InitExecuteCommand(
                () -> initializeDrive(true),
                () -> driveOrientationBased(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble(), robotCentric.getAsBoolean()),
                this
        );
    }

    public Command driveWhilstRotatingToTarget(DoubleSupplier x, DoubleSupplier y, Pose2d target, BooleanSupplier robotCentric) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> driveWithTarget(x.getAsDouble(), y.getAsDouble(), target, robotCentric.getAsBoolean()),
                interrupt -> {},
                () -> false,
                this
        );
    }

    public Command rotateToTarget(Pose2d target) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> driveWithTarget(0, 0, target, false),
                interrupt -> {
                },
                ROTATION_CONTROLLER::atGoal,
                this
        ).withTimeout(5);
    }

    public void stop() {
        for (SwerveModule currentModule : MODULES)
            currentModule.stop();
    }

    public Rotation2d getGyroHeading() {
        final double inputtedHeading = MathUtil.inputModulus(GYRO.getYaw(), -180, 180);
        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public void setGyroHeading(Rotation2d heading) {
        GYRO.setGyroYaw(heading.getDegrees());
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getSelfRelativeVelocity(), RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation());
    }

    public void periodicallyUpdateFromOdometry() {
        final int odometryUpdates = GYRO.getInputs().threadGyroYawDegrees.length;

        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(GYRO.getInputs().threadGyroYawDegrees[i]);
        }

        POSE_ESTIMATOR.addOdometryObservations(swerveWheelPositions, gyroRotations, OdometryThread.getInstance().getLatestTimestamps());
    }

    private void driveOrientationBased(double xPower, double yPower, double thetaPower, boolean robotCentric) {
        if (robotCentric)
            driveSelfRelative(xPower, yPower, thetaPower);
        else
            driveFieldRelative(xPower, yPower, thetaPower);
    }

    private void driveWithTarget(double xPower, double yPower, Pose2d target, boolean robotCentric) {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        final Rotation2d targetAngle = getAngleFromPoseToPose(RobotContainer.POSE_ESTIMATOR.getCurrentPose(), target);

        final double controllerOutput = Units.degreesToRadians(
                ROTATION_CONTROLLER.calculate(
                        currentAngle.getDegrees(),
                        targetAngle.getDegrees()
                ));

        if (robotCentric)
            driveSelfRelative(xPower, yPower, controllerOutput);
        else
            driveFieldRelative(xPower, yPower, controllerOutput);
    }

    private void driveFieldRelative(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation()
//                .minus(Rotation2d.fromDegrees(180))
        );

        driveSelfRelative(speeds);
    }

    private void driveSelfRelative(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        driveSelfRelative(speeds);
    }

    private void driveSelfRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);

        if (Optimizations.isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_MPS);

        for (int i = 0; i < MODULES.length; i++)
            MODULES[i].setTargetState(swerveModuleStates[i]);
    }

    private void initializeDrive(boolean openLoop) {
        for (SwerveModule currentModule : MODULES)
            currentModule.setOpenLoop(openLoop);

        ROTATION_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
    }

    private SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            swerveModulePositions[i] = MODULES[i].getOdometryPosition(odometryUpdateIndex);

        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                POSE_ESTIMATOR::getCurrentPose,
                POSE_ESTIMATOR::resetPose,

                this::getSelfRelativeVelocity,
                this::driveSelfRelative,

                HOLONOMIC_PATH_FOLLOWER_CONFIG,
                Mirrorable::isRedAlliance,
                this
        );

        PathfindingCommand.warmupCommand().schedule();
    }

    private ChassisSpeeds proportionalSpeedToMps(ChassisSpeeds chassisSpeeds) {
        return new ChassisSpeeds(
                proportionalPowerToMps(chassisSpeeds.vxMetersPerSecond, MAX_SPEED_MPS),
                proportionalPowerToMps(chassisSpeeds.vyMetersPerSecond, MAX_SPEED_MPS),
                chassisSpeeds.omegaRadiansPerSecond
        );
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    private SwerveModuleState[] getModuleTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getTargetState();

        return states;
    }

    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double difference = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        return ChassisSpeeds.discretize(chassisSpeeds, difference);
    }
}
