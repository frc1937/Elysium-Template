package frc.lib.generic.hardware.motor;

public enum MotorSignal {
    CURRENT(1), POSITION(4), VELOCITY(5), VOLTAGE(0), TEMPERATURE(2), CLOSED_LOOP_TARGET(3);

    final int id;

    MotorSignal(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}
