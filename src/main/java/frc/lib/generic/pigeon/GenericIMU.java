package frc.lib.generic.pigeon;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class GenericIMU extends WPI_PigeonIMU implements Pigeon {
    private final String name;

    public GenericIMU(String name, int deviceNumber) {
        super(deviceNumber);

        this.name = name;
    }

    @Override
    public void resetConfigurations() {
        super.configFactoryDefault();
    }

    public void setGyroYaw(double yawDegrees) {
        super.setYaw(yawDegrees);
    }

    @Override
    public double getYaw() {
        return super.getYaw();
    }
}
