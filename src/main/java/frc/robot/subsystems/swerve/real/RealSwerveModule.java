package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.SwerveModuleIO;

public class RealSwerveModule implements SwerveModuleIO {
    @Override
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getCurrentAngle());
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return null;
    }


}
