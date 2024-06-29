package frc.robot.subsystems.swerve.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;

import static frc.lib.math.Conversions.*;
import static frc.robot.subsystems.swerve.simulation.SimulatedSwerveConstants.WHEEL_DIAMETER;

public class SimulatedSwerveModule extends SwerveModuleIO {
    private final SimpleMotorSimulation driveMotor, steerMotor;

    public SimulatedSwerveModule(String name, SimpleMotorSimulation driveMotor, SimpleMotorSimulation steerMotor) {
        super(name);

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setInput(MotorProperties.ControlMode.POSITION, angle.getRotations());
        System.out.println("hsa changed angle");
    }

    @Override
    protected void setTargetVelocity(double velocityMetresPerSecond) {
        driveMotor.setInput(MotorProperties.ControlMode.VELOCITY, mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER));
        System.out.println("Has put velocity");
    }

    @Override
    protected void refreshInputs(SwerveModuleInputsAutoLogged swerveModuleInputs) {
        swerveModuleInputs.driveDistanceMeters = rotationsToMetres(driveMotor.getPositionRotations(), WHEEL_DIAMETER);
        swerveModuleInputs.driveVoltage = driveMotor.getVoltage();
        swerveModuleInputs.driveVelocityMetersPerSecond = rpsToMps(driveMotor.getVelocityRotationsPerSecond(), WHEEL_DIAMETER);

        swerveModuleInputs.steerAngleDegrees = rotationsToDegrees(steerMotor.getPositionRotations());
        swerveModuleInputs.steerVoltage = steerMotor.getVoltage();

        swerveModuleInputs.odometryUpdatesDriveDistanceMeters = new double[]{swerveModuleInputs.driveDistanceMeters};
        swerveModuleInputs.odometryUpdatesSteerAngleDegrees = new double[]{swerveModuleInputs.steerAngleDegrees};

    }
}
