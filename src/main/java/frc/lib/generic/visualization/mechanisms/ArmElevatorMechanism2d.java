package frc.lib.generic.visualization.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createDefaultRoot;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createElevatorOutline;

public class ArmElevatorMechanism2d {
    private final String name;
    private final Mechanism2d armElevatorMechanism;
    private final MechanismRoot2d
            root,
            targetRoot;
    private MechanismLigament2d
            currentLigament,
            targetLigament;

    public ArmElevatorMechanism2d(String name, double armLength) {
        this.name = "Mechanism/" + name;
        this.armElevatorMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, 20);

        this.root = createDefaultRoot("armElevatorRoot", armElevatorMechanism);
        this.targetRoot = createDefaultRoot("armElevatorTargetRoot", armElevatorMechanism);

        createCurrentLigaments(armLength);
        createTargetLigaments(armLength);
        createOutline();
    }

    public void updateCurrentPosition(double position) {
        root.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + position);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public void updateTargetPosition(double targetPosition) {
        targetRoot.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + targetPosition);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public void updateCurrentAngle(Rotation2d currentAngle) {
        currentLigament.setAngle(currentAngle);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public void updateTargetAngle(Rotation2d targetAngle) {
        targetLigament.setAngle(targetAngle);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    private void createCurrentLigaments(double elevatorLength) {
        currentLigament = new MechanismLigament2d("armElevatorLigament", elevatorLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, RED);
        root.append(currentLigament);
    }

    private void createTargetLigaments(double elevatorLength) {
        targetLigament = new MechanismLigament2d("targetArmElevatorLigament", elevatorLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, BLUE);
        targetRoot.append(targetLigament);
    }

    private void createOutline() {
        createElevatorOutline(armElevatorMechanism);
    }
}