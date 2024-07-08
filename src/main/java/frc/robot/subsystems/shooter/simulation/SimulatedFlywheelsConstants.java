package frc.robot.subsystems.shooter.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.FlywheelSimulation;
import frc.robot.subsystems.flywheel.simulation.SimulatedFlywheel;
import frc.robot.subsystems.shooter.FlywheelsConstants;
import frc.robot.subsystems.shooter.SingleFlywheelIO;

import java.util.Optional;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.ofReplayable;

public class SimulatedFlywheelsConstants extends FlywheelsConstants {
    private static final DCMotor FLYWHEEL_MOTOR_GEARBOX = DCMotor.getNeoVortex(1);

    private static final FlywheelSimulation
            LEFT_MOTOR = new FlywheelSimulation(FLYWHEEL_MOTOR_GEARBOX, 1, 0.017),
            RIGHT_MOTOR = new FlywheelSimulation(FLYWHEEL_MOTOR_GEARBOX, 1, 0.015);

    private static final MotorProperties.Slot
            LEFT_SLOT = new MotorProperties.Slot(12, 0, 0),
            RIGHT_SLOT = new MotorProperties.Slot(10, 0, 0);

    private static final MotorConfiguration configuration = new MotorConfiguration();

    static {
        configureMotor(LEFT_MOTOR, true, LEFT_SLOT);
        configureMotor(RIGHT_MOTOR, false, RIGHT_SLOT);
    }

    private static void configureMotor(FlywheelSimulation motor, boolean invert, MotorProperties.Slot slot) {
        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 80;
        configuration.statorCurrentLimit = 100;

        configuration.slot0 = slot;

        motor.configure(configuration);
    }

    @Override
    protected Optional<SingleFlywheelIO[]> getFlywheels() {
        return ofReplayable(() -> new SingleFlywheelIO[] {
                new SimulatedSingleFlywheel("LeftSim", LEFT_MOTOR, LEFT_MOTOR_INVERT, LEFT_FLYWHEEL_DIAMETER),
                new SimulatedSingleFlywheel(RIGHT_MOTOR, "RightSim", RIGHT_MOTOR_INVERT, RIGHT_FLYWHEEL_DIAMETER)
        });
    }
}
