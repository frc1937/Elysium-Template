package frc.robot.subsystems.swerve.simulationswerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.GyroSimulation;
import frc.lib.generic.simulation.SimpleMotorSimulation;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

import java.util.Optional;

public class SimulationSwerveConstants extends SwerveConstants {
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

    private static final SimpleMotorSimulation[] DRIVE_MOTORS = new SimpleMotorSimulation[]{
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_RIGHT_DRIVE_MOTOR,
            REAR_LEFT_DRIVE_MOTOR,
            REAR_RIGHT_DRIVE_MOTOR
    };

    private static final SimpleMotorSimulation[] STEER_MOTORS = new SimpleMotorSimulation[]{
            FRONT_LEFT_STEER_MOTOR,
            FRONT_RIGHT_STEER_MOTOR,
            REAR_LEFT_STEER_MOTOR,
            REAR_RIGHT_STEER_MOTOR
    };

    private static void configureDriveMotor(SimpleMotorSimulation driveMotor) {
        final MotorConfiguration config = new MotorConfiguration();
        driveMotor.configure(config);
    }

    private static void configureSteerMotor(SimpleMotorSimulation steerMotor) {
        final MotorConfiguration config = new MotorConfiguration();

        config.slot0 = new MotorProperties.Slot(25, 0, 0, 0, 0, 0);
        config.closedLoopContinousWrap = true;

        steerMotor.configure(config);
    }

    private static final Optional<SwerveModuleIO[]> MODULES_IO = ofReplayable(() -> new SwerveModuleIO[]{
            new SimulationSwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_STEER_MOTOR, "FrontRight"),
            new SimulationSwerveModule(FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_STEER_MOTOR, "FrontLeft"),
            new SimulationSwerveModule(REAR_RIGHT_DRIVE_MOTOR, REAR_RIGHT_STEER_MOTOR, "RearRight"),
            new SimulationSwerveModule(REAR_LEFT_DRIVE_MOTOR, REAR_LEFT_STEER_MOTOR, "RearLeft")
    });

    static final GyroSimulation GYRO = new GyroSimulation();

    static {
        if(GlobalConstants.CURRENT_MODE != GlobalConstants.Mode.REPLAY) {
            for (int i = 0; i < DRIVE_MOTORS.length; i++) {
                configureDriveMotor(DRIVE_MOTORS[i]);
                configureSteerMotor(STEER_MOTORS[i]);
            }
        }
    }

    @Override
    protected Optional<WPI_PigeonIMU> getPigeon() {
        return Optional.empty();
    }

    @Override
    protected Optional<SwerveModuleIO[]> getModulesIO() {
        return MODULES_IO;
    }
}
