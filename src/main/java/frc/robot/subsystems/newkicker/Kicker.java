//package frc.robot.subsystems.newkicker;
//
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.lib.generic.motor.Motor;
//import frc.lib.generic.motor.MotorProperties;
//import frc.lib.generic.sensors.Sensor;
//import frc.robot.GlobalConstants;
//
//import static frc.robot.GlobalConstants.CURRENT_MODE;
//
//public class Kicker extends SubsystemBase {
//    private final Motor motor = generateMotor();
//    private final Sensor beamBreaker = generateBeamBreaker();
//
//    public Command setKickerPercentageOutput(double percentageOutput) {
//        return Commands.startEnd(
//                () -> motor.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentageOutput),
//                this::stop,
//                this
//        );
//    }
//
//    public boolean doesSeeNote() {
//        if (beamBreaker == null) return true;
//
//        return beamBreaker.get() == 0;
//    }
//
//    private void stop() {
//        motor.stopMotor();
//    }
//
//    private Sensor generateBeamBreaker() {
//        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
//            return KickerConstants.BEAM_BREAKER;
//        }
//
//        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
//            return null;
//        }
//
//        return new Sensor("NOTE_DETECTOR");
//    }
//
//    private Motor generateMotor() {
//        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
//            return KickerConstants.REAL_MOTOR;
//        }
//
//        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
//            return KickerConstants.SIMULATION_MOTOR;
//        }
//
//        return new Motor("Kicker");
//    }
//}