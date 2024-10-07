package frc.lib.generic.hardware.motor.hardware.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;

public class GenericSparkBase extends Motor {
    private final CANSparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkPIDController sparkController;

    public GenericSparkBase(String name, int deviceId) {
        super(name);

        spark = new CANSparkFlex(deviceId, CANSparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();
        sparkController = spark.getPIDController();
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        spark.setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
    }


    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, 32767);
    }
}
