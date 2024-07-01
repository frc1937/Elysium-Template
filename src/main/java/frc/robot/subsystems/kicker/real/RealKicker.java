package frc.robot.subsystems.kicker.real;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerInputsAutoLogged;

import static frc.robot.subsystems.kicker.real.RealKickerConstants.BEAM_BREAKER;
import static frc.robot.subsystems.kicker.real.RealKickerConstants.MOTOR;

public class RealKicker extends KickerIO {
    @Override
    protected void setPercentageOutput(double percentageOutput) {
        MOTOR.set(ControlMode.PercentOutput, percentageOutput);
    }

    @Override
    protected void stop() {
        MOTOR.stopMotor();
    }

    @Override
    protected void refreshInputs(KickerInputsAutoLogged inputs) {
        inputs.doesSeeNote = !BEAM_BREAKER.get();
        inputs.voltage = MOTOR.getMotorOutputVoltage() * 12;
    }
}
