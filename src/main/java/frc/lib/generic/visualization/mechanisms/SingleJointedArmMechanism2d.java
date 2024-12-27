package frc.lib.generic.visualization.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createDefaultRoot;

public class SingleJointedArmMechanism2d {
    private final String name;
    private final Mechanism2d armMechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentAngleLigament,
            targetAngleLigament;

    public SingleJointedArmMechanism2d(String name, double armLength) {
        this.name = "Mechanism/" + name;
        this.armMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = createDefaultRoot("armRoot", armMechanism);

        createCurrentLigament(armLength);
        createTargetLigament(armLength);
    }

    public void updateCurrentAngle(Rotation2d currentAngle) {
        currentAngleLigament.setAngle(currentAngle);
        Logger.recordOutput(name, armMechanism);
    }

    public void updateTargetAngle(Rotation2d targetAngle) {
        targetAngleLigament.setAngle(targetAngle);
        Logger.recordOutput(name, armMechanism);
    }

    private void createCurrentLigament(double armLength) {
        currentAngleLigament = new MechanismLigament2d("armLigament", armLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, BLUE);
        root.append(currentAngleLigament);
    }

    private void createTargetLigament(double armLength) {
        targetAngleLigament = new MechanismLigament2d("targetArmLigament", armLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, GRAY);
        root.append(targetAngleLigament);
    }
}