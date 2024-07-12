package frc.lib.generic.pigeon;

import org.littletonrobotics.junction.AutoLog;

public class PigeonIO {
    @AutoLog
    public static class GenericPigeonInputs {
        public double gyroYawDegrees = 0;
        public double gyroRollDegrees = 0;
        public double gyroPitchDegrees = 0;

        public double[] odometryUpdatesTimestamp = new double[0];
        public double[] odometryUpdatesYawDegrees = new double[0];
        public double[] odometryUpdatesPitchDegrees = new double[0];
        public double[] odometryUpdatesRollDegrees = new double[0];
    }
}
