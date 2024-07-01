package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelInputsAutoLogged flywheelInputs = new FlywheelInputsAutoLogged();
    private final FlywheelIO[] flywheels = FlywheelIO.generateFlywheels();


    @Override
    public void periodic() {

    }

}
