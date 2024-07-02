package frc.robot.subsystems.flywheel.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.FlywheelSimulation;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class SimulatedFlywheelConstants extends FlywheelConstants {
    private static final DCMotor FLYWHEEL_MOTOR_GEARBOX = DCMotor.getNeoVortex(1);

    private static final FlywheelSimulation
            LEFT_MOTOR = new FlywheelSimulation(FLYWHEEL_MOTOR_GEARBOX, 1, 0.017),
            RIGHT_MOTOR = new FlywheelSimulation(FLYWHEEL_MOTOR_GEARBOX, 1, 0.015);

    private static final MotorProperties.Slot
        LEFT_SLOT = new MotorProperties.Slot(12, 0, 0),
        RIGHT_SLOT = new MotorProperties.Slot(10, 0, 0);

    private static final MotorConfiguration configuration = new MotorConfiguration();

    static {
        configureMotor(LEFT_MOTOR, LEFT_MOTOR_INVERT, LEFT_SLOT);
        configureMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERT, RIGHT_SLOT);
    }

    private static void configureMotor(FlywheelSimulation motor, boolean invert, MotorProperties.Slot slot) {
        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 80;
        configuration.statorCurrentLimit = 100;

        configuration.slot0 = slot;

        motor.configure(configuration);
    }

    public static FlywheelIO[] getFlywheels() {
        return new FlywheelIO[]{
                new SimulatedFlywheel(LEFT_MOTOR, "LeftSim", LEFT_MOTOR_INVERT, LEFT_FLYWHEEL_DIAMETER),
                new SimulatedFlywheel(RIGHT_MOTOR, "RightSim", RIGHT_MOTOR_INVERT, RIGHT_FLYWHEEL_DIAMETER)
        };
    }
}
