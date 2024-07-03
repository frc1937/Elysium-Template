package frc.robot.subsystems.swerve_old.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve_old.OLDSwerveModuleInputsAutoLogged;
import frc.robot.subsystems.swerve_old.SwerveModuleIO;

import static frc.lib.math.Conversions.*;
import static frc.robot.subsystems.swerve_old.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve_old.simulation.SimulatedSwerveConstants.WHEEL_DIAMETER;

public class SimulatedSwerveModule extends SwerveModuleIO {
    private final SimpleMotorSimulation driveMotor, steerMotor;

    public SimulatedSwerveModule(String name, SimpleMotorSimulation driveMotor, SimpleMotorSimulation steerMotor) {
        super(name);

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    @Override
    protected void setTargetVelocity(double velocityMetresPerSecond, boolean openLoop) {
        driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, 12 * mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER) / MAX_SPEED_MPS);
    }

    @Override
    public void stop() {
        driveMotor.stop();
        steerMotor.stop();
    }

    @Override
    protected void refreshInputs(OLDSwerveModuleInputsAutoLogged swerveModuleInputs) {
        swerveModuleInputs.driveDistanceMeters = rotationsToMetres(driveMotor.getPositionRotations(), WHEEL_DIAMETER);
        swerveModuleInputs.driveVoltage = driveMotor.getVoltage();
        swerveModuleInputs.driveVelocityMetersPerSecond = rpsToMps(driveMotor.getVelocityRotationsPerSecond(), WHEEL_DIAMETER);

        swerveModuleInputs.steerAngleDegrees = rotationsToDegrees(steerMotor.getPositionRotations());
        swerveModuleInputs.steerVoltage = steerMotor.getVoltage();

        swerveModuleInputs.odometryUpdatesDriveDistanceMeters = new double[]{swerveModuleInputs.driveDistanceMeters};
        swerveModuleInputs.odometryUpdatesSteerAngleDegrees = new double[]{swerveModuleInputs.steerAngleDegrees};
    }
}
