//package frc.robot.subsystems.newkicker;
//
//import edu.wpi.first.math.system.plant.DCMotor;
//import frc.lib.generic.motor.Motor;
//import frc.lib.generic.motor.MotorConfiguration;
//import frc.lib.generic.motor.MotorProperties;
//import frc.lib.generic.motor.hardware.GenericTalonSRX;
//import frc.lib.generic.sensors.Sensor;
//import frc.lib.generic.sensors.hardware.DigitalInput;
//import frc.lib.generic.simulation.SimpleMotorSimulation;
//
//public class KickerConstants {
//    public static final Sensor BEAM_BREAKER = new DigitalInput("NOTE_DETECTOR", 0);
//
//    public static final Motor REAL_MOTOR = new GenericTalonSRX("KICKER", 8);
//    public static final SimpleMotorSimulation SIMULATION_MOTOR = new SimpleMotorSimulation(
//            DCMotor.getNeo550(1),
//            1.0,
//            0.003);
//
//    static {
//        configureMotor();
//    }
//
//    private static void configureMotor() {
//        MotorConfiguration configuration = new MotorConfiguration();
//
//        configuration.idleMode = MotorProperties.IdleMode.BRAKE;
//
//        REAL_MOTOR.configure(configuration);
//        SIMULATION_MOTOR.configure(configuration);
//    }
//}
