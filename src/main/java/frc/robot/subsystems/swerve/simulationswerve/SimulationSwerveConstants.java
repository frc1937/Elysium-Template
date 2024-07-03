package frc.robot.subsystems.swerve.simulationswerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.lib.generic.simulation.GyroSimulation;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;

import java.util.Optional;

public class SimulationSwerveConstants extends SwerveConstants {


    private static final Optional<SwerveModuleIO[]> MODULES_IO = ofReplayable(() -> new SwerveModuleIO[]{
            new SimulationSwerveModule(SimulationSwerveModuleConstants.FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, "FrontRight"),
            new SimulationSwerveModule(SimulationSwerveModuleConstants.FRONT_LEFT_SWERVE_MODULE_CONSTANTS, "FrontLeft"),
            new SimulationSwerveModule(SimulationSwerveModuleConstants.REAR_RIGHT_SWERVE_MODULE_CONSTANTS, "RearRight"),
            new SimulationSwerveModule(SimulationSwerveModuleConstants.REAR_LEFT_SWERVE_MODULE_CONSTANTS, "RearLeft")
    });

    static final GyroSimulation GYRO = new GyroSimulation();

    @Override
    protected Optional<WPI_PigeonIMU> getPigeon() {
        return Optional.empty();
    }

    @Override
    protected Optional<SwerveModuleIO[]> getModulesIO() {
        return MODULES_IO;
    }
}
