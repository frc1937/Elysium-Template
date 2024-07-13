package frc.lib.generic.sensors.hardware;

import frc.lib.generic.sensors.Sensor;
import frc.lib.generic.sensors.SensorInputsAutoLogged;

public class DigitalInput extends Sensor {
    private final edu.wpi.first.wpilibj.DigitalInput digitalInput;

    public DigitalInput(String name, int id) {
        super(name);

        digitalInput = new edu.wpi.first.wpilibj.DigitalInput(id);
    }

    @Override
    public void refreshInputs(SensorInputsAutoLogged inputs) {
        inputs.currentValue = digitalInput.get() ? 1 : 0;
    }
}
