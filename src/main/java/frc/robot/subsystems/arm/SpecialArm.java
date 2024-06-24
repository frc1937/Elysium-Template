package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.generic.Properties;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.SingleJointedArmSimulation;
import frc.lib.util.TunableNumber;

public class SpecialArm {
    private final SingleJointedArmSimulation armSimulation;

    private final TunableNumber setpoint = new TunableNumber("target arm", 20);

    public SpecialArm() {
        armSimulation = new SingleJointedArmSimulation(
                DCMotor.getNeoVortex(1),
                1,
                0.56,
                0.20,
                Rotation2d.fromDegrees(-20),
                Rotation2d.fromDegrees(150),
                false
        );

        MotorConfiguration configuration = new MotorConfiguration();

        configuration.slot0 = new MotorProperties.Slot(100, 0, 0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        configuration.slotToUse = 0;

        armSimulation.configure(configuration);
    }

    public void reachSetpoint() {
        armSimulation.setOutput(MotorProperties.ControlMode.VOLTAGE, 10);

        SmartDashboard.putNumber("PositionDegs", Units.rotationsToDegrees(armSimulation.getPositionRotations()));
        SmartDashboard.putNumber("VelocityDegsPS", Units.rotationsToDegrees(armSimulation.getVelocityRotationsPerSecond()));

    }
}
