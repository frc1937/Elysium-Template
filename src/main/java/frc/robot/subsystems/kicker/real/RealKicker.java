package frc.robot.subsystems.kicker.real;

import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerInputsAutoLogged;

import static frc.robot.subsystems.kicker.real.RealKickerConstants.BEAM_BREAKER;
import static frc.robot.subsystems.kicker.real.RealKickerConstants.MOTOR;

public class RealKicker extends KickerIO {
    @Override
    protected void setPercentageOutput(double percentageOutput) {
        MOTOR.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentageOutput);
    }

    @Override
    protected void stop() {
        MOTOR.stopMotor();
    }

    @Override
    protected void refreshInputs(KickerInputsAutoLogged inputs) {
        inputs.doesSeeNote = !BEAM_BREAKER.get();
        inputs.voltage = MOTOR.getVoltage();
    }
}
