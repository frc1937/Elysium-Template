package frc.lib.generic.visualization.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;
import static frc.lib.generic.visualization.mechanisms.MechanismConstants.GRAY;


public class MechanismUtilities {
    protected static MechanismRoot2d createDefaultRoot(String name, Mechanism2d mechanism) {
        return mechanism.getRoot(name, DEFAULT_ROOT_X, DEFAULT_ROOT_Y);
    }

    protected static void createElevatorOutline(Mechanism2d mechanism) {
        final MechanismRoot2d outlineRoot = mechanism.getRoot("outlineRoot", 1, 1);

        final MechanismLigament2d
                outlineTop = new MechanismLigament2d("outlineTop", 8, -90, DEFAULT_LINE_WIDTH, GRAY),
                outlineLeft = new MechanismLigament2d("outlineLeft", 18, 90, DEFAULT_LINE_WIDTH, GRAY),
                outlineRight = new MechanismLigament2d("outlineRight", 18, -90, DEFAULT_LINE_WIDTH, GRAY);

        outlineTop.append(outlineRight);
        outlineRoot.append(outlineLeft);
        outlineLeft.append(outlineTop);
    }
}