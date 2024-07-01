package frc.robot.subsystems.flywheel.simulation;

import frc.lib.generic.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class SimulatedFlywheel extends FlywheelIO {
    public SimulatedFlywheel(SimpleMotorSimulation motor, String name) {
        super(name);
    }

}
