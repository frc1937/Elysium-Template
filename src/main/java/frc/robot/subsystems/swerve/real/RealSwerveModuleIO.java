package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.robot.poseestimation.poseestimator.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;

import java.util.Queue;

import static frc.lib.math.Optimizations.removeCouplingFromRevolutions;

public class RealSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final RealSwerveModuleConstants moduleConstants;
    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(RealSwerveModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(RealSwerveModuleConstants.ENABLE_FOC);

    RealSwerveModuleIO(RealSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.moduleConstants = moduleConstants;

        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(moduleConstants.steerPositionSignal);
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(moduleConstants.drivePositionSignal);
    }

    @Override
    protected void refreshInputs(SwerveModuleInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::rotationsToDegrees).toArray();
        inputs.steerVoltage = moduleConstants.steerVoltageSignal.getValue();

        inputs.driveDistanceMeters = toDriveDistance(moduleConstants.drivePositionSignal.getValue());
        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = toDriveDistance(moduleConstants.driveVelocitySignal.getValue());
        inputs.driveVoltage = moduleConstants.driveVoltageSignal.getValue();

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                RealSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                moduleConstants.steerVelocitySignal.getValue(),
                RealSwerveModuleConstants.COUPLING_RATIO,
                RealSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                RealSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double targetVelocityRevolutionsPerSeconds = Conversions.metresToRotations(targetVelocityMetersPerSecond, RealSwerveModuleConstants.WHEEL_DIAMETER_METERS);

        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                targetVelocityRevolutionsPerSeconds,
                Rotation2d.fromRotations(moduleConstants.steerVelocitySignal.getValue()),
                RealSwerveModuleConstants.COUPLING_RATIO
        );

        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    private double getAngleDegrees() {
        return Conversions.rotationsToDegrees(moduleConstants.steerPositionSignal.getValue());
    }

    private double toDriveDistance(double revolutions) {
        return Conversions.rotationsToMetres(revolutions, RealSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                moduleConstants.steerVelocitySignal,
                moduleConstants.driveVelocitySignal,
                moduleConstants.driveStatorCurrentSignal
        );
    }
}
