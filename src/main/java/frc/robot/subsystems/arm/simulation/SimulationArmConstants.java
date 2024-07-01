package frc.robot.subsystems.arm.simulation;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SingleJointedArmSimulation;
import frc.lib.generic.simulation.mechanisms.SingleJointedArmMechanism2d;

public class SimulationArmConstants {
    static final double PITCH_GEAR_RATIO = 1.0 / 149;

    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);

    static final double
            PITCH_KS = 0.053988,
            PITCH_KV = 37,
            PITCH_KA = 0,
            PITCH_KG = 0.04366,
            PITCH_KP = 100,
            PITCH_KI = 0.0,
            PITCH_KD = 0.0,
            PITCH_MAX_VELOCITY = 0.5,
            PITCH_MAX_ACCELERATION = 0.5;


    static final DCMotor ARM_GEARBOX = DCMotor.getNeoVortex(1);

    static final SingleJointedArmSimulation ARM_MOTOR = new SingleJointedArmSimulation(
            ARM_GEARBOX,
            150.0,
            0.2,
            0.03,
            Rotation2d.fromDegrees(-20),
            Rotation2d.fromDegrees(120),
            true,
            true
    );

    static final SingleJointedArmMechanism2d ARM_MECHANISM =
            new SingleJointedArmMechanism2d("ArmMechanism", new Color8Bit(Color.kRed));

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration motorConfiguration = new MotorConfiguration();

        motorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        motorConfiguration.slot0 = new MotorProperties.Slot(
                PITCH_KP,
                PITCH_KI,
                PITCH_KD,
                PITCH_KV,
                PITCH_KA,
                PITCH_KS,
                PITCH_KG,
                GravityTypeValue.Arm_Cosine
        );

        ARM_MOTOR.configure(motorConfiguration);

        ARM_MOTOR.update();
    }
}
