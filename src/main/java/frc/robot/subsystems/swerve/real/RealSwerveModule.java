package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.Properties;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.lib.util.threads.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;

import java.util.Queue;

import static frc.lib.math.Conversions.*;
import static frc.robot.GlobalConstants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.real.RealSwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.real.RealSwerveConstants.WHEEL_DIAMETER;

public class RealSwerveModule extends SwerveModuleIO {
    private boolean OPEN_LOOP = true;

    private final Motor driveMotor, steerMotor;
    private final Encoder steerEncoder;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public RealSwerveModule(Motor driveMotor, Motor steerMotor, Encoder steerEncoder, String name) {
        super(name);

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;

        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerEncoder.getRawStatusSignal(Properties.SignalType.POSITION));
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getRawStatusSignal(Properties.SignalType.POSITION));
    }

    @Override
    public void periodic() {
        steerMotor.setMotorEncoderPosition(degreesToRotations(getAngleDegrees()));
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setInput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    @Override
    public void setTargetVelocity(double velocityMetresPerSecond) {
        if (OPEN_LOOP) {
            final double targetPowerOpenLoop = VOLTAGE_COMPENSATION_SATURATION * velocityMetresPerSecond / MAX_SPEED_MPS;
            driveMotor.setInput(MotorProperties.ControlMode.VOLTAGE, targetPowerOpenLoop);
        } else {
            final double targetVelocityRPSClosedLoop = Conversions.mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER);
            driveMotor.setInput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPSClosedLoop);
        }
    }

    @Override
    protected void refreshInputs(SwerveModuleInputsAutoLogged swerveModuleInputs) {
        driveMotor.refreshStatusSignals(Properties.SignalType.VELOCITY, Properties.SignalType.POSITION);

        swerveModuleInputs.steerAngleDegrees = getAngleDegrees();
        swerveModuleInputs.steerVoltage = steerMotor.getVoltage();

        swerveModuleInputs.driveVelocityMetersPerSecond = driveMotor.getSystemVelocity();
        swerveModuleInputs.driveDistanceMeters = rotationsToMetres(driveMotor.getSystemPosition(), WHEEL_DIAMETER);
        swerveModuleInputs.driveVoltage = driveMotor.getVoltage();

        swerveModuleInputs.odometryUpdatesDriveDistanceMeters =
                drivePositionQueue.stream().mapToDouble((position) -> rotationsToMetres(position, WHEEL_DIAMETER)).toArray();

        swerveModuleInputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::rotationsToDegrees).toArray();
    }

    private double getAngleDegrees() {
        return rotationsToDegrees(steerEncoder.getEncoderPosition());
    }
}
