package frc.robot.subsystems.shooter.real;

import frc.lib.generic.motor.Motor;
import frc.robot.subsystems.shooter.SingleFlywheelIO;
import frc.robot.subsystems.shooter.SingleFlywheelInputsAutoLogged;

public class RealSingleFlywheel extends SingleFlywheelIO {
    private final Motor motor;

    public RealSingleFlywheel(String name, Motor motor) {
        super(name);

        this.motor = motor;
    }

    @Override
    protected void flywheelPeriodic() {

    }

    @Override
    protected void stop() {

    }

    @Override
    protected void refreshInputs(SingleFlywheelInputsAutoLogged singleFlywheelInputs) {

    }
}
