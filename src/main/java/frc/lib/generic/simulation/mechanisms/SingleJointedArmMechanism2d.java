package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.createDefaultRoot;

public class SingleJointedArmMechanism2d {
    private final String name;
    private final Mechanism2d armMechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentAngleLigament,
            targetAngleLigament;

    public SingleJointedArmMechanism2d(String name, double armLength, Rotation2d defaultAngle) {
        this.name = name;
        this.armMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = createDefaultRoot("armRoot", armMechanism);

        createCurrent(armLength, defaultAngle);
        createTarget(armLength, defaultAngle);
    }

    public void updateCurrentAngle(Rotation2d currentAngle) {
        currentAngleLigament.setAngle(currentAngle);
        Logger.recordOutput(name, armMechanism);
    }

    public void updateTargetAngle(Rotation2d targetAngle) {
        targetAngleLigament.setAngle(targetAngle);
        Logger.recordOutput(name, armMechanism);
    }

    private void createCurrent(double armLength, Rotation2d defaultAngle) {
        currentAngleLigament = new MechanismLigament2d("armLigament", armLength, defaultAngle.getDegrees(), DEFAULT_LINE_WIDTH, BLUE);
        root.append(currentAngleLigament);
    }

    private void createTarget(double armLength, Rotation2d defaultAngle) {
        targetAngleLigament = new MechanismLigament2d("targetArmLigament", armLength, defaultAngle.getDegrees(), DEFAULT_LINE_WIDTH, GRAY);
        root.append(targetAngleLigament);
    }
}