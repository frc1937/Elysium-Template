package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.ABSOLUTE_ARM_ENCODER;
import static frc.robot.subsystems.arm.ArmConstants.ARM_MECHANISM;
import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;
import static frc.robot.subsystems.arm.ArmConstants.SYSID_CONFIG;
import static frc.robot.utilities.ShooterPhysicsCalculations.calculateShootingAngle;
import static frc.robot.utilities.ShooterPhysicsCalculations.zoneInOnShootingAngle;

public class Arm extends GenericSubsystem {
    public Arm() {
        setName("Arm Subsystem");
        ARM_MOTOR.setMotorEncoderPosition(ABSOLUTE_ARM_ENCODER.getEncoderPosition());
    }

    public boolean hasReachedTarget() {
        Logger.recordOutput("IsArmReady ", ARM_MOTOR.isAtPositionSetpoint());
        return ARM_MOTOR.isAtPositionSetpoint();
    }

    public double getVelocity() {
        return ARM_MOTOR.getSystemVelocity();
    }

    public Command setTargetPhysicsBasedPositionNoZoneIn(Pose3d targetPose, double targetVelocityRPS) {
        double[] targetAngleRotations = new double[1];

        return new FunctionalCommand(
                () -> targetAngleRotations[0] =
                        calculateShootingAngle(targetPose, Conversions.rpsToMps(targetVelocityRPS, FlywheelsConstants.RIGHT_FLYWHEEL_DIAMETER)).getRotations(),
                () -> setMotorTargetPosition(Rotation2d.fromRotations(targetAngleRotations[0])),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }


    public Command setTargetPhysicsBasedPosition(Pose3d targetPose, double targetVelocityRPS) {
        double[] targetAngleRotations = new double[1];

        return new FunctionalCommand(
                () -> targetAngleRotations[0] =
                        zoneInOnShootingAngle(targetPose, Conversions.rpsToMps(targetVelocityRPS, FlywheelsConstants.RIGHT_FLYWHEEL_DIAMETER)).getRotations(),
                () -> setMotorTargetPosition(Rotation2d.fromRotations(targetAngleRotations[0] + Units.degreesToRotations(3))),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command setTargetPosition(Rotation2d targetPosition) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setMotorTargetPosition(targetPosition),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command setTargetPositionSupplier(DoubleSupplier targetPosition) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setMotorTargetPosition(Rotation2d.fromDegrees(targetPosition.getAsDouble())),
                interrupted -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }


    @Override
    public void periodic() {
        ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition()));
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
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
        ARM_MECHANISM.updateTargetAngle(targetPosition);
    }
}
