package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.createDefaultRoot;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.createElevatorOutline;

public class ElevatorMechanism2d {
    private final String name;
    private final Mechanism2d elevatorMechanism;
    private final MechanismRoot2d
            root,
            targetRoot;

    public ElevatorMechanism2d(String name, double elevatorLength) {
        this.name = name;
        this.elevatorMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, 20);

        this.root = createDefaultRoot("elevatorRoot", elevatorMechanism);
        this.targetRoot = createDefaultRoot("elevatorTargetRoot", elevatorMechanism);

        createCurrentLigament(elevatorLength);
        createTargetLigament(elevatorLength);
        createOutlineLigament();
    }

    public void updateCurrentPosition(double position) {
        root.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + position);
        Logger.recordOutput(name, elevatorMechanism);
    }

    public void updateTargetPosition(double targetPosition) {
        targetRoot.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + targetPosition);
        Logger.recordOutput(name, elevatorMechanism);
    }

    private void createCurrentLigament(double elevatorLength) {
        final MechanismLigament2d currentRightLigament = new MechanismLigament2d("elevatorRightLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, RED);
        final MechanismLigament2d currentLeftLigament = new MechanismLigament2d("elevatorLeftLigament", elevatorLength, 180, DEFAULT_LINE_WIDTH, RED);

        root.append(currentRightLigament);
        root.append(currentLeftLigament);
    }

    private void createTargetLigament(double elevatorLength) {
        final MechanismLigament2d targetRightLigament = new MechanismLigament2d("targetRightLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, BLUE);
        final MechanismLigament2d targetLeftLigament = new MechanismLigament2d("targetLeftLigament", elevatorLength, 180, DEFAULT_LINE_WIDTH, BLUE);

        targetRoot.append(targetRightLigament);
        targetRoot.append(targetLeftLigament);
    }

    private void createOutlineLigament() {
        createElevatorOutline(elevatorMechanism);
    }
}