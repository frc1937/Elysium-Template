package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.real.RealSwerveModule;

import static frc.robot.subsystems.swerve.real.RealSwerveConstants.*;

public class Swerve {

    SwerveModuleIO[] swerveModules = new SwerveModuleIO[4];

    public Swerve() {
        swerveModules[0] = new RealSwerveModule(FL_DRIVE_MOTOR, FL_STEER_MOTOR, FL_STEER_ENCODER);
    }
}
