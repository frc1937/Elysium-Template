package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.createDefaultRoot;


public class DoubleJointedArmMechanism2d {
    private final String name;
    private final Mechanism2d doubleJointedArmMechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentShoulderLigament,
            targetShoulderLigament,
            currentElbowLigament,
            targetElbowLigament;

    public DoubleJointedArmMechanism2d(String name, double shoulderLength, double elbowLength) {
        this.name = name;
        this.doubleJointedArmMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = createDefaultRoot("DoubleJointedArmRoot", doubleJointedArmMechanism);

        createCurrentLigament(shoulderLength, elbowLength);
        createTargetLigament(shoulderLength, elbowLength);
    }

    public void updateCurrentAngle(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
        currentShoulderLigament.setAngle(shoulderAngle);
        currentElbowLigament.setAngle(elbowAngle.minus(shoulderAngle));

        Logger.recordOutput(name, doubleJointedArmMechanism);
    }

    public void updateTargetAngle(Rotation2d targetShoulderAngle, Rotation2d targetElbowAngle) {
        targetShoulderLigament.setAngle(targetShoulderAngle);
        targetElbowLigament.setAngle(targetElbowAngle.minus(targetShoulderAngle));

        Logger.recordOutput(name, doubleJointedArmMechanism);
    }

    private void createCurrentLigament(double shoulderLength, double elbowLength) {
        currentShoulderLigament = new MechanismLigament2d("currentShoulderLigament", shoulderLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, BLUE);
        currentElbowLigament = new MechanismLigament2d("currentElbowLigament", elbowLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, DARK_BLUE);

        currentShoulderLigament.append(currentElbowLigament);
        root.append(currentShoulderLigament);
    }

    private void createTargetLigament(double shoulderLength, double elbowLength) {
        targetShoulderLigament = new MechanismLigament2d("targetShoulderLigament", shoulderLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, GRAY);
        targetElbowLigament = new MechanismLigament2d("targetElbowLigament", elbowLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, DARK_GRAY);

        targetShoulderLigament.append(targetElbowLigament);
        root.append(targetShoulderLigament);
    }
}