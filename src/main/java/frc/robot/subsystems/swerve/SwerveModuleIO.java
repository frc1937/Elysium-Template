package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    void setTargetState(SwerveModuleState state);

    Rotation2d getCurrentAngle();
}
