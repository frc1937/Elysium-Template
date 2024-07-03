package frc.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SimpleMotorSimulation;

import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.real.RealSwerveConstants.DRIVE_GEAR_RATIO;

public class SimulationSwerveModuleConstants {

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;

    private static final DCMotor
            DRIVE_MOTOR_GEARBOX = DCMotor.getFalcon500(1),
            STEER_MOTOR_GEARBOX = DCMotor.getNeo550(1);

    private static final SimpleMotorSimulation
            FRONT_LEFT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            FRONT_RIGHT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_LEFT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_RIGHT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);
    private static final SimpleMotorSimulation
            FRONT_LEFT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            FRONT_RIGHT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_LEFT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_RIGHT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    static final SimulationSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_STEER_MOTOR
    ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR
            );

    final SimpleMotorSimulation driveMotor, steerMotor;

    private SimulationSwerveModuleConstants(SimpleMotorSimulation driveMotor, SimpleMotorSimulation steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        configureDriveMotor();
        configureSteerMotor();
    }

    private void configureDriveMotor() {
        final MotorConfiguration config = new MotorConfiguration();
        driveMotor.configure(config);
    }

    private void configureSteerMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.slot0 = new MotorProperties.Slot(5, 0, 0, 0, 0, 0);
        config.ClosedLoopContinousWrap = true;

        steerMotor.configure(config);
    }
}
