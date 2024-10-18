package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.robot.utilities.ShooterPhysicsCalculations;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.arm.ArmConstants.ABSOLUTE_ARM_ENCODER;
import static frc.robot.subsystems.arm.ArmConstants.ARM_MECHANISM;
import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;
import static frc.robot.subsystems.arm.ArmConstants.SYSID_CONFIG;

public class Arm extends GenericSubsystem {
    public Arm() {
        setName("Arm Subsystem");
        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    public boolean hasReachedTarget() {
        Logger.recordOutput("IsArmReady: ", ARM_MOTOR.isAtPositionSetpoint());

        return ARM_MOTOR.isAtPositionSetpoint();
    }

    public double getVelocity() {
        return ARM_MOTOR.getSystemVelocity();
    }

    public void UNSAFE_setVoltage(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public Command setTargetPhysicsBasedPosition(ShooterPhysicsCalculations calculations, Pose3d targetPose, double tangentialVelocity) {
        double[] target = new double[1];

        return new FunctionalCommand(
                () -> target[0] = calculations.getOptimalShootingAngleRadians(POSE_ESTIMATOR.getCurrentPose(), targetPose, tangentialVelocity),
                () -> setMotorTargetPosition(Rotation2d.fromRadians(target[0]).plus(Rotation2d.fromDegrees(6))),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command setTargetPosition(Rotation2d targetPosition) {
        return new FunctionalCommand(
                () -> {},
                () -> setMotorTargetPosition(targetPosition),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    @Override
    public void periodic() {
        ARM_MECHANISM.updateMechanism(Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition()));
    }

    public double getTargetAngleRotations() {
        return ARM_MOTOR.getClosedLoopTarget();
    }

    public double getCurrentAngleRotations() {
        return ARM_MOTOR.getSystemPosition();
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_CONFIG;
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .voltage(Volts.of(ARM_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(ARM_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(ARM_MOTOR.getSystemVelocity()));
    }

    @Override
    public void sysIdDrive(double voltage) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    private void setMotorTargetPosition(Rotation2d targetPosition) {
        System.out.println("SETting arm position to " + targetPosition.getDegrees());
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,  targetPosition.getRotations());
        ARM_MECHANISM.setTargetAngle(targetPosition);
    }
}
