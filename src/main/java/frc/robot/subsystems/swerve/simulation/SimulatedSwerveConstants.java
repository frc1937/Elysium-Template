package frc.robot.subsystems.swerve.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.GyroSimulation;
import frc.lib.generic.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

public class SimulatedSwerveConstants extends SwerveConstants {
    static final GyroSimulation GYRO = new GyroSimulation();

    static final double DRIVE_GEAR_RATIO = 6.75;
    static final double ANGLE_GEAR_RATIO = (150.0 / 7.0);

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;

    private static final DCMotor DRIVE_MOTOR_GEARBOX = DCMotor.getFalcon500(1);
    private static final DCMotor STEER_MOTOR_GEARBOX = DCMotor.getNeo550(1);

    private static final SimpleMotorSimulation
            FL_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            FR_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            RL_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            RR_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);

    private static final SimpleMotorSimulation
            FL_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            FR_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            RL_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            RR_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, ANGLE_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    static {
        for (SimpleMotorSimulation motor : new SimpleMotorSimulation[]{FL_DRIVE_MOTOR, FR_DRIVE_MOTOR, RL_DRIVE_MOTOR, RR_DRIVE_MOTOR}) {
            configureDriveMotor(motor);
        }

        for (SimpleMotorSimulation motor : new SimpleMotorSimulation[]{FL_STEER_MOTOR, FR_STEER_MOTOR, RL_STEER_MOTOR, RR_STEER_MOTOR}) {
            configureSteerMotor(motor);
        }
    }

    @Override
    protected SwerveModuleIO[] getSwerveModules() {
        return new SwerveModuleIO[]{
                new SimulatedSwerveModule("FL", FL_DRIVE_MOTOR, FL_STEER_MOTOR),
                new SimulatedSwerveModule("FR", FR_DRIVE_MOTOR, FR_STEER_MOTOR),
                new SimulatedSwerveModule("RL", RL_DRIVE_MOTOR, RL_STEER_MOTOR),
                new SimulatedSwerveModule("RR", RR_DRIVE_MOTOR, RR_STEER_MOTOR)
        };
    }

    private static void configureDriveMotor(SimpleMotorSimulation driveMotor) {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.slot0 = new MotorProperties.Slot(0.4, 0, 0, 1, 1, 0.5);
        configuration.dutyCycleCloseLoopRampPeriod = 0.1;

        driveMotor.configure(configuration);
    }

    private static void configureSteerMotor(SimpleMotorSimulation steerMotor) {
        MotorConfiguration config = new MotorConfiguration();

        config.slot0 = new MotorProperties.Slot(15, 0, 0);
        config.ClosedLoopContinousWrap = true;

        steerMotor.configure(config);
    }

}
