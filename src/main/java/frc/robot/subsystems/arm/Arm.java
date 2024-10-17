package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.scurve.InputParameter;
import frc.lib.scurve.OutputParameter;
import frc.lib.scurve.UpdateResult;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotContainer.S_CURVE_GENERATOR;
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

    public Command setContinousTargetPosition(DoubleSupplier targetRadians) {
        return new FunctionalCommand(
                () -> {},//resetMotor(Units.radiansToRotations(targetRadians.getAsDouble())),
                () -> setMotorTargetPosition(Rotation2d.fromRadians(targetRadians.getAsDouble())),
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

    public Command setSCurvePosition(Rotation2d targetPosition) {
        InputParameter[] input = new InputParameter[]{resetProfile(targetPosition.getRotations())};
        OutputParameter[] output = new OutputParameter[]{new OutputParameter()};
        UpdateResult[] result = new UpdateResult[1];

        double[] t = {0};

        return new FunctionalCommand(
                () -> {
                    input[0] = resetProfile(targetPosition.getRotations());
                    output[0] = new OutputParameter();
                },
                () -> {
                    result[0] = S_CURVE_GENERATOR.update(input[0], output[0]);

                    input[0] = result[0].input_parameter;
                    output[0] = result[0].output_parameter;

                    double kG = 0.25408;
                    double kS = 0.1205;

                    double pidOutput = 48 * (output[0].new_position - ARM_MOTOR.getSystemPosition());

                    if (pidOutput < kS && pidOutput > -kS) {
                        pidOutput = kS * Math.signum(pidOutput);
                    }

                    double ff = kS * Math.signum(output[0].new_velocity) +
                            15.625 * output[0].new_velocity +
                            0.85843 * output[0].new_acceleration +
                            kG * Math.cos(ARM_MOTOR.getSystemPosition() * 2 * Math.PI);

                    Logger.recordOutput("ARM_PID_OUTPUT", pidOutput);
                    Logger.recordOutput("ARM_FF_OUTPUT", ff);

                    Logger.recordOutput("ARM_TARGET", output[0].new_position);
                    Logger.recordOutput("ARM_ERROR_DEG", 360*(output[0].new_position - ARM_MOTOR.getSystemPosition()));

                    ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, pidOutput + ff);

                    t[0]+=0.02;
                },
                interrupted -> {
                },
                () -> {
                    return false;
                },
                this
        );
    }

    private InputParameter resetProfile(double targetPosition) {
        return new InputParameter(
                ARM_MOTOR.getSystemPosition(),
                ARM_MOTOR.getSystemVelocity(),
                ARM_MOTOR.getSystemAcceleration(),
                targetPosition,
                0.5,
                0.5,
                1.3
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
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,  targetPosition.getRotations());
        ARM_MECHANISM.setTargetAngle(targetPosition);
    }
}
