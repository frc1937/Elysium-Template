// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MeasureUtils {
    public static <U extends Unit<U>> Measure<U> interpolate(Measure<U> startValue, Measure<U> endValue, double t) {
        double magnitude = Interpolator.forDouble().interpolate(startValue.baseUnitMagnitude(), endValue.baseUnitMagnitude(), t);
        return startValue.unit().ofBaseUnits(magnitude);
    }
}
