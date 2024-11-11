package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.PID;
import frc.lib.math.Optimizations;
import frc.lib.util.mirrorable.Mirrorable;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.AutoLogOutput;

import static frc.lib.math.Conversions.proportionalPowerToMps;
import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.swerve.SwerveConstants.GYRO;
import static frc.robot.subsystems.swerve.SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATION_CONTROLLER;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;

public class Swerve extends GenericSubsystem {
    private static final PID translationController = new PID(HOLONOMIC_PATH_FOLLOWER_CONFIG.translationConstants);
    private static final PID rotationController = new PID(HOLONOMIC_PATH_FOLLOWER_CONFIG.rotationConstants);

    private double lastTimestamp = Timer.getFPGATimestamp();

    public Swerve() {
        configurePathPlanner();
    }

    public void stop() {
        for (SwerveModule currentModule : MODULES)
            currentModule.stop();
    }

    public void runWheelCharacterization(double rotationSpeed) {
        driveSelfRelative(0, 0, rotationSpeed);
    }

    public Rotation2d getGyroHeading() {
        final double inputtedHeading =
//                MathUtil.inputModulus(
                GYRO.getYaw()
//                , -180, 180)
                ;
        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public double[] getDriveWheelPositionsRadians() {
        double[] positions = new double[4];

        for (int i = 0; i < MODULES.length; i++) {
            positions[i] = MODULES[i].getDriveWheelPositionRadians();
        }

        return positions;
    }

    public void setGyroHeading(Rotation2d heading) {
        GYRO.setGyroYaw(heading.getDegrees());
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic() {
        final double[] odometryUpdatesYawDegrees = GYRO.getInputs().threadGyroYawDegrees;
        final int odometryUpdates = odometryUpdatesYawDegrees.length;

        if (OdometryThread.getInstance().getLatestTimestamps().length == 0) return;

        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(odometryUpdatesYawDegrees[i]);
        }

        POSE_ESTIMATOR.addOdometryObservations(
                        swerveWheelPositions,
                        gyroRotations,
                        OdometryThread.getInstance().getLatestTimestamps()
                );
    }

    protected void driveOrientationBased(double xPower, double yPower, double thetaPower, boolean robotCentric) {
        if (robotCentric)
            driveSelfRelative(xPower, yPower, thetaPower);
        else
            driveFieldRelative(xPower, yPower, thetaPower);
    }

    protected void driveWithTarget(double xPower, double yPower, Pose2d target, boolean robotCentric) {
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

    protected void driveToPose(Pose2d target) {
        Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        driveFieldRelative(
                translationController.calculate(
                        currentPose.getX(),
                        target.getX()
                ),
                translationController.calculate(
                        currentPose.getY(),
                        target.getY()
                ),
                rotationController.calculate(
                        currentPose.getRotation().getDegrees(),
                        target.getRotation().getDegrees()
                )
        );
    }

    protected void driveFieldRelative(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation());

        driveSelfRelative(speeds);
    }

    protected void driveSelfRelative(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        driveSelfRelative(speeds);
    }

    protected void driveSelfRelative(ChassisSpeeds chassisSpeeds) {
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

    protected void initializeDrive(boolean openLoop) {
        for (SwerveModule currentModule : MODULES)
            currentModule.setOpenLoop(openLoop);

        ROTATION_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
    }

    protected SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[MODULES.length];

        for (int i = 0; i < MODULES.length; i++) {
            swerveModulePositions[i] = MODULES[i].getOdometryPosition(odometryUpdateIndex);
            if (swerveModulePositions[i] == null) return null;
        }

        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    protected void configurePathPlanner() {
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

    protected ChassisSpeeds proportionalSpeedToMps(ChassisSpeeds chassisSpeeds) {
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
    protected SwerveModuleState[] getModuleTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getTargetState();

        return states;
    }

    protected ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double difference = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        return ChassisSpeeds.discretize(chassisSpeeds, difference);
    }
}
