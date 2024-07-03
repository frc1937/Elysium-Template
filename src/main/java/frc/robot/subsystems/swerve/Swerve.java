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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.mirrorable.Mirrorable;
import frc.lib.util.mirrorable.MirrorablePose2d;
import frc.lib.util.mirrorable.MirrorableRotation2d;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.GlobalConstants.ODOMETRY_LOCK;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class Swerve extends SubsystemBase {
    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveIO swerveIO = SwerveIO.generateIO();

    private final SwerveConstants constants = SwerveConstants.generateConstants();
    private final SwerveModuleIO[] modulesIO;

    private double lastTimestamp = 0;

    public Swerve() {
        modulesIO = getModulesIO();
        configurePathPlanner();
    }

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        updateAllInputs();
        ODOMETRY_LOCK.unlock();

        updatePoseEstimatorStates();
        updateNetworkTables();
    }

    public void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return new FunctionalCommand(
                () -> {initializeDrive(true);},
                () -> selfRelativeDrive(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble()),
                (interrupt) -> {},
                () -> false,
                this
        );
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    public Rotation2d getHeading() {
        final double inputtedHeading = MathUtil.inputModulus(swerveInputs.gyroYawDegrees, -180, 180);
        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public void setGyroHeading(Rotation2d heading) {
        swerveIO.setGyroHeading(heading);
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getSelfRelativeVelocity(), POSE_ESTIMATOR.getCurrentPose().getRotation());
    }

    /**
     * Checks if the robot is at a pose.
     *
     * @param pose2d a pose, relative to the blue alliance driver station's right corner
     * @return whether the robot is at the pose
     */
    public boolean atPose(MirrorablePose2d pose2d) {
        final Pose2d mirroredPose = pose2d.get();
        return atXAxisPosition(mirroredPose.getX()) && atYAxisPosition(mirroredPose.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return atTranslationPosition(POSE_ESTIMATOR.getCurrentPose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return atTranslationPosition(POSE_ESTIMATOR.getCurrentPose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean atAngle(MirrorableRotation2d angle) {
        final boolean atTargetAngle = Math.abs(angle.get().minus(POSE_ESTIMATOR.getCurrentPose().getRotation()).getDegrees()) < SwerveConstants.ROTATION_TOLERANCE_DEGREES;
        final boolean isAngleStill = Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
        Logger.recordOutput("Swerve/AtTargetAngle/isStill", isAngleStill);
        Logger.recordOutput("Swerve/AtTargetAngle/atTargetAngle", atTargetAngle);
        return atTargetAngle/* && isAngleStill*/;
    }

    public SwerveModulePosition[] getWheelPositions() {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];
        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getOdometryPosition(modulesIO[i].getLastOdometryUpdateIndex());
        return swerveModulePositions;
    }

    public void runWheelRadiusCharacterization(double omegaRadsPerSec) {
        selfRelativeDrive(new ChassisSpeeds(0, 0, omegaRadsPerSec));
    }

    /**
     * Locks the swerve, so it'll be hard to move it. This will make the modules look in the middle of a robot in an "x" shape.
     */
    void lockSwerve() {
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modulesIO[0].setTargetState(left);
        modulesIO[1].setTargetState(right);
        modulesIO[2].setTargetState(right);
        modulesIO[3].setTargetState(left);
    }


    void initializeDrive(boolean closedLoop) {
        setClosedLoop(closedLoop);
        resetRotationController();
    }

    void resetRotationController() {
        ROTATION_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
    }

    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setOpenLoop(closedLoop);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle, relative to the blue alliance's forward position
     */
    void fieldRelativeDrive(double xPower, double yPower, MirrorableRotation2d targetAngle) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        Logger.recordOutput("Swerve/AnglePID/TargetAngle", MathUtil.inputModulus(targetAngle.get().getDegrees(), 0, 360));
        Logger.recordOutput("Swerve/AnglePID/AngleSetpoint", MathUtil.inputModulus(ROTATION_CONTROLLER.getSetpoint().position, 0, 360));
        Logger.recordOutput("Swerve/AnglePID/CurrentAngle", MathUtil.inputModulus(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees(), 0, 360));

        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the robot's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     */
    void selfRelativeDrive(double xPower, double yPower, MirrorableRotation2d targetAngle) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        Logger.recordOutput("Stuff/TargetAngle", MathUtil.inputModulus(targetAngle.get().getDegrees(), 0, 360));
        Logger.recordOutput("Stuff/AngleSetpoint", MathUtil.inputModulus(ROTATION_CONTROLLER.getSetpoint().position, 0, 360));
        Logger.recordOutput("Stuff/CurrentAngle", MathUtil.inputModulus(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees(), 0, 360));

        selfRelativeDrive(speeds);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);

        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_MPS);

        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double difference = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;
        return ChassisSpeeds.discretize(chassisSpeeds, difference);
    }

    private void updatePoseEstimatorStates() {
        final int odometryUpdates = swerveInputs.odometryUpdatesYawDegrees.length;
        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(swerveInputs.odometryUpdatesYawDegrees[i]);
        }

        POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, swerveInputs.odometryUpdatesTimestamp);
    }

    private SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];
        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getOdometryPosition(odometryUpdateIndex);
        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    private void updateAllInputs() {
        swerveIO.refreshInputs(swerveInputs);
        Logger.processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> POSE_ESTIMATOR.getCurrentPose(),
//                (pose) -> RobotContainer.POSE_ESTIMATOR.resetPose(RobotContainer.POSE_ESTIMATOR.getCurrentPose()),
                (pose) -> {
                },
                this::getSelfRelativeVelocity,
                this::selfRelativeDrive,
                HOLONOMIC_PATH_FOLLOWER_CONFIG,
                Mirrorable::isRedAlliance,
                this
        );
        PathfindingCommand.warmupCommand().schedule();
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    private double calculateProfiledAngleSpeedToTargetAngle(MirrorableRotation2d targetAngle) {
        final Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation();
        return Units.degreesToRadians(ROTATION_CONTROLLER.calculate(currentAngle.getDegrees(), targetAngle.get().getDegrees()));
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getDriveRelativeAngle());
    }

    private Rotation2d getDriveRelativeAngle() {
        final Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation();
        return Mirrorable.isRedAlliance() ? currentAngle.rotateBy(Rotation2d.fromDegrees(180)) : currentAngle;
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * MAX_SPEED_MPS,
                yPower * MAX_SPEED_MPS,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * MAX_ROTATION_RAD_PER_S
        );
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/Rot", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/X", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/Y", getSelfRelativeVelocity().vyMetersPerSecond);
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private SwerveModuleIO[] getModulesIO() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new SwerveModuleIO[]{
                    new SwerveModuleIO("FrontLeft"),
                    new SwerveModuleIO("FrontRight"),
                    new SwerveModuleIO("RearLeft"),
                    new SwerveModuleIO("RearRight")
            };
        }

        return constants.getModulesIO().get();
    }
}