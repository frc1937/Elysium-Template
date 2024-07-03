package frc.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SimpleMotorSimulation;
import frc.lib.math.Conversions;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_ROTATION_RAD_PER_S;

public class SimulationSwerveModuleIO extends SwerveModuleIO {
    private final SimpleMotorSimulation driveMotor, steerMotor;

    SimulationSwerveModuleIO(SimulationSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
    }

    protected void setTargetVelocity(double targetVelocityMetersPerSecond, boolean openLoop) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                steerMotor.getVelocityRotationsPerSecond(),
                0,
                MAX_ROTATION_RAD_PER_S,
                SimulationSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        Logger.recordOutput(getLoggingPath() + "driveVoltage", driveMotor.getVoltage());
        driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    @Override
    protected void stop() {
        driveMotor.stop();
        steerMotor.stop();
    }

    @Override
    protected void refreshInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = Conversions.rotationsToDegrees(steerMotor.getPositionRotations());
        inputs.steerVoltage = steerMotor.getVoltage();

        inputs.driveDistanceMeters = Conversions.rotationsToMetres(driveMotor.getPositionRotations(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityMetersPerSecond = Conversions.rpsToMps(driveMotor.getVelocityRotationsPerSecond(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVoltage = driveMotor.getVoltage();

        inputs.odometryUpdatesSteerAngleDegrees = new double[]{inputs.steerAngleDegrees};
        inputs.odometryUpdatesDriveDistanceMeters = new double[]{inputs.driveDistanceMeters};
    }
}
