package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class FlywheelsConstants {
    private static final DCMotor FLYWHEEL_MOTOR_GEARBOX = DCMotor.getNeoVortex(1);

    private static final Motor LEFT_FLYWHEEL_MOTOR = MotorFactory.createSpark("LEFT_FLYWHEEL", 28, MotorProperties.SparkType.FLEX);
    private static final Motor RIGHT_FLYWHEEL_MOTOR = MotorFactory.createSpark("RIGHT_FLYWHEEL", 15, MotorProperties.SparkType.FLEX);

    private static final MotorProperties.Slot
            LEFT_SLOT = new MotorProperties.Slot(5, 0, 0,  0.10904, 0.025022, 0.22468),
            RIGHT_SLOT = new MotorProperties.Slot(5, 0, 0,  0.10457, 0.037788, 0.05658),

    SIMULATION_SLOT = new MotorProperties.Slot(12+0.30681, 0, 0,  0.10432, 0.35682, 0.0019243);

    public static final double
            LEFT_FLYWHEEL_DIAMETER = 0.07,
            RIGHT_FLYWHEEL_DIAMETER = 0.1;

    protected static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(2),
            Second.of(9)
    );

    static {
        configureMotor(LEFT_FLYWHEEL_MOTOR, false, LEFT_SLOT);
        configureMotor(RIGHT_FLYWHEEL_MOTOR, true, RIGHT_SLOT);
    }

    private static void configureMotor(Motor motor, boolean invert, MotorProperties.Slot slot) {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 40;
        configuration.statorCurrentLimit = 40;

//        configuration.profiledTargetAcceleration = 40;
//        configuration.profiledJerk = 30;

        configuration.slot0 = slot;
//todo: flywheels are oscilating likec razy. insepct.
        configuration.closedLoopTolerance = 0.5;

        configuration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.FLYWHEEL,
                FLYWHEEL_MOTOR_GEARBOX,
                1,
                0.017
        );

        configuration.simulationSlot = SIMULATION_SLOT;

        motor.configure(configuration);

        motor.setupSignalUpdates(MotorSignal.VOLTAGE);
        motor.setupSignalUpdates(MotorSignal.TEMPERATURE);
        motor.setupSignalUpdates(MotorSignal.VELOCITY);
        motor.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }

    protected static SingleFlywheel[] flywheels = {
            new SingleFlywheel(LEFT_FLYWHEEL_MOTOR, LEFT_FLYWHEEL_DIAMETER),
            new SingleFlywheel(RIGHT_FLYWHEEL_MOTOR, RIGHT_FLYWHEEL_DIAMETER)
    };
}
