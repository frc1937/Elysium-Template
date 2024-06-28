package frc.robot.subsystems.swerve.real;

import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.GenericCanCoder;
import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.GenericTalonFX;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

public class RealSwerveConstants {
    static final Motor FL_STEER_MOTOR = new GenericSpark(0, MotorProperties.SparkType.MAX);
    static final Motor FL_DRIVE_MOTOR = new GenericTalonFX(14);
    static final Encoder FL_STEER_ENCODER = new GenericCanCoder(0);

    static final MotorConfiguration steerMotorConfiguration = new MotorConfiguration();

    static {


        configureSteerMotor(FL_STEER_MOTOR);
    }

    private static void configureSteerEncoder(Encoder steerEncoder, double angleOffset) {

    }

    private static void configureDriveMotor(Motor driveMotor) {

    }

    private static void configureSteerMotor(Motor steerMotor) {
        steerMotorConfiguration.slot0 = new MotorProperties.Slot(1, 0, 0, 1, 0, 0);

//        steerMotorConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
//        steerMotorConfiguration.inverted = ANGLE_MOTOR_INVERT;
//        steerMotorConfiguration.idleMode = ANGLE_NEUTRAL_MODE;
//
//        steerMotorConfiguration.conversionFactor = ANGLE_GEAR_RATIO;
//
//        steerMotor.setMotorPosition(getCurrentAngle().getRotations());
//        steerMotor.setSignalUpdateFrequency(Properties.SignalType.POSITION, 50);
    }
}
