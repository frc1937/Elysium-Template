package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.math.Optimizations;
import frc.robot.GlobalConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.lib.math.Conversions.proportionalPowerToMps;
import static frc.lib.math.Conversions.proportionalPowerToRotation;
import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.robot.GlobalConstants.ODOMETRY_LOCK;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.real.RealSwerveConstants.SWERVE_KINEMATICS;

public class Swerve extends SubsystemBase {
    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveConstants constants = SwerveConstants.generateConstants();

    private final SwerveModuleIO[] swerveModules = getSwerveModules();
    private final SwerveIO swerve = SwerveIO.generateSwerve();

    private double lastTimestamp = 0;

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        refreshAllInputs();
        ODOMETRY_LOCK.unlock();

        updateOdometryFromInputs();
        refreshRotationController();
    }

    public Command driveWhilstRotatingToTarget(DoubleSupplier x, DoubleSupplier y, Pose2d target) {
        return new FunctionalCommand(
                this::resetRotationController,
                () -> driveFieldRelative(x.getAsDouble(), y.getAsDouble(), target),
                (interrupt) -> {},
                ROTATION_CONTROLLER::atGoal,
                this
        );
    }

    public Command rotateToTarget(Pose2d target) {
        return new FunctionalCommand(
                this::resetRotationController,
                () -> driveFieldRelative(0, 0, target),
                (interrupt) -> {},
                ROTATION_CONTROLLER::atGoal,
                this
        );
    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return new FunctionalCommand(
                () -> {
                },
                () -> driveFieldRelative(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble()),
                (interrupt) -> {
                },
                () -> false,
                this
        );
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> swerve.setGyroHeading(Rotation2d.fromDegrees(0)), this);
    }

    public void setGyroHeading(Rotation2d heading) {
        swerve.setGyroHeading(heading);
    }

    public Rotation2d getGyroHeading() {
        final double boundedHeading = MathUtil.inputModulus(swerveInputs.gyroYawDegrees, -180, 180);
        return Rotation2d.fromDegrees(boundedHeading);
    }

    public void stop() {
        for (SwerveModuleIO currentModule : swerveModules)
            currentModule.stop();
    }

    public ChassisSpeeds getSelfRelativeSpeeds() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    private void driveFieldRelative(double xPower, double yPower, Pose2d target) {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        final Rotation2d targetAngle = getAngleFromPoseToPose(RobotContainer.POSE_ESTIMATOR.getCurrentPose(), target);

        final double controllerOutput = ROTATION_CONTROLLER.calculate(
                currentAngle.getRadians(),
                targetAngle.getRadians()
        );

        driveFieldRelative(xPower, yPower, controllerOutput);
    }

    private void driveFieldRelative(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation());

        driveSelfRelative(speeds);
    }

    private void driveSelfRelative(ChassisSpeeds speeds) {
        if (Optimizations.isStill(speeds)) {
            stop();
            return;
        }

        speeds = Optimizations.discretize(speeds, lastTimestamp);

        SwerveModuleState[] targetStates = SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setTargetState(targetStates[i]);
        }

        lastTimestamp = Timer.getFPGATimestamp();
    }

    private SwerveModuleIO[] getSwerveModules() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new SwerveModuleIO[]{
                    new SwerveModuleIO("FrontLeft"),
                    new SwerveModuleIO("FrontRight"),
                    new SwerveModuleIO("RearLeft"),
                    new SwerveModuleIO("RearRight")
            };
        }

        return constants.getSwerveModules();
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++)
            states[i] = swerveModules[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++)
            states[i] = swerveModules[i].getTargetState();

        return states;
    }

    private ChassisSpeeds proportionalSpeedToMps(ChassisSpeeds chassisSpeeds) {
        return new ChassisSpeeds(
                proportionalPowerToMps(chassisSpeeds.vxMetersPerSecond, MAX_SPEED_MPS),
                proportionalPowerToMps(chassisSpeeds.vyMetersPerSecond, MAX_SPEED_MPS),
                proportionalPowerToRotation(chassisSpeeds.omegaRadiansPerSecond, MAX_ROTATION_RAD_PER_S)
        );
    }

    private void refreshAllInputs() {
        swerve.refreshInputs(swerveInputs);
        Logger.processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO swerveModule : swerveModules) {
            swerveModule.periodic();
        }
    }

    private void updateOdometryFromInputs() {
        final int odometryUpdates = swerveInputs.odometryUpdatesYawDegrees.length;

        final Rotation2d[] gyroUpdates = new Rotation2d[odometryUpdates];
        final SwerveDriveWheelPositions[] swerveDriveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];

        if (odometryUpdates == 0) return;

        for (int i = 0; i < odometryUpdates; i++) {
            gyroUpdates[i] = Rotation2d.fromDegrees(swerveInputs.odometryUpdatesYawDegrees[i]);
            swerveDriveWheelPositions[i] = getWheelPositions(i);
        }

        RobotContainer.POSE_ESTIMATOR.updatePoseEstimatorStates(
                swerveDriveWheelPositions,
                gyroUpdates,
                swerveInputs.odometryUpdatesTimestamp
        );
    }

    private SwerveDriveWheelPositions getWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModulePositions[i] = swerveModules[i].getOdometryPosition(odometryUpdateIndex);
        }

        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    private void refreshRotationController() {
        ROTATION_CONTROLLER.setP(ROTATION_KP.get());
        ROTATION_CONTROLLER.setConstraints(new TrapezoidProfile.Constraints(
                ROTATION_MAX_VELOCITY.get(),
                ROTATION_MAX_ACCELERATION.get())
        );
    }

    private void resetRotationController() {
        ROTATION_CONTROLLER.reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getRadians());
    }
}
