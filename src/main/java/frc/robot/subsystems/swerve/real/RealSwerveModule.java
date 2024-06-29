package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.robot.subsystems.swerve.SwerveModuleIO;

import static frc.robot.GlobalConstants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.real.RealSwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.real.RealSwerveConstants.WHEEL_DIAMETER;

public class RealSwerveModule extends SwerveModuleIO {
    private boolean OPEN_LOOP = true;

    private final Motor driveMotor, steerMotor;
    private final Encoder steerEncoder;

    public RealSwerveModule(Motor driveMotor, Motor steerMotor, Encoder steerEncoder) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
    }

    @Override
    public void periodic() {
        steerMotor.setMotorEncoderPosition(getCurrentAngle().getRotations());
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    @Override
    public void setTargetVelocity(double velocityMetresPerSecond) {
        if (OPEN_LOOP) {
            final double targetPowerOpenLoop = VOLTAGE_COMPENSATION_SATURATION * velocityMetresPerSecond / MAX_SPEED_MPS;
            driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, targetPowerOpenLoop);
        } else {
            final double targetVelocityRPSClosedLoop = Conversions.mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER);
            driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPSClosedLoop);
        }
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerEncoder.getEncoderPosition());
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged swerveModuleInputs) {
        swerveModuleInputs.steerAngleDegrees = steerMotor.getMotorPosition();
        swerveModuleInputs.steerVoltage = steerMotor.getMotorOutput();
        swerveModuleInputs.driveVelocityMetersPerSecond = driveMotor.getMotorVelocity();
        swerveModuleInputs.driveDistanceMeters = driveMotor.getMotorPosition();
        swerveModuleInputs.driveVoltage = driveMotor.getMotorOutput();
    }

}
