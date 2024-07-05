package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.robot.poseestimation.poseestimator.SparkOdometryThread;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;

import java.util.Queue;

import static frc.lib.generic.Properties.SignalType.POSITION;
import static frc.lib.generic.Properties.SignalType.VELOCITY;
import static frc.lib.math.Conversions.degreesToRotations;
import static frc.lib.math.Conversions.rotationsToDegrees;
import static frc.lib.math.Conversions.rotationsToMetres;
import static frc.robot.GlobalConstants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.WHEEL_DIAMETER;

public class RealSwerveModule extends SwerveModuleIO {
    private final Motor driveMotor, steerMotor;
    private final Encoder steerEncoder;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public RealSwerveModule(Motor driveMotor, Motor steerMotor, Encoder steerEncoder, String name) {
        super(name);

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;

        steerPositionQueue = SparkOdometryThread.getInstance().registerSignal(steerEncoder::getEncoderPosition);
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor::getSystemPosition);
    }

    @Override
    public void modulePeriodic() {
        steerMotor.setMotorEncoderPosition(degreesToRotations(getAngleDegrees()));
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    @Override
    public void setTargetVelocity(double velocityMetresPerSecond, boolean openLoop) {
        if (openLoop) {
            final double targetPowerOpenLoop = VOLTAGE_COMPENSATION_SATURATION * (velocityMetresPerSecond / MAX_SPEED_MPS);
            driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, targetPowerOpenLoop);
        } else {
            final double targetVelocityRPSClosedLoop = Conversions.mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER);
            driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPSClosedLoop);
        }
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    @Override
    protected void refreshInputs(SwerveModuleInputsAutoLogged swerveModuleInputs) {
        driveMotor.refreshStatusSignals(VELOCITY, POSITION);
        steerEncoder.refreshStatusSignals(POSITION);

        swerveModuleInputs.steerAngleDegrees = getAngleDegrees();
        swerveModuleInputs.steerVoltage = steerMotor.getVoltage();

        swerveModuleInputs.driveVelocityMetersPerSecond = driveMotor.getSystemVelocity();
        swerveModuleInputs.driveDistanceMeters = rotationsToMetres(driveMotor.getSystemPosition(), WHEEL_DIAMETER);
        swerveModuleInputs.driveVoltage = driveMotor.getVoltage();

        swerveModuleInputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble((position) -> rotationsToMetres(position, WHEEL_DIAMETER)).toArray();
        swerveModuleInputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::rotationsToDegrees).toArray();

        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    private double getAngleDegrees() {
        return rotationsToDegrees(steerEncoder.getEncoderPosition());
    }
}
