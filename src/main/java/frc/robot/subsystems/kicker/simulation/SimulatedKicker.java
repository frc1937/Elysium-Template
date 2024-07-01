package frc.robot.subsystems.kicker.simulation;

import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerInputsAutoLogged;

import static frc.robot.subsystems.kicker.simulation.SimulatedKickerConstants.MOTOR;

public class SimulatedKicker extends KickerIO {
    @Override
    protected void setPercentageOutput(double percentageOutput) {
        MOTOR.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentageOutput);
    }

    @Override
    protected void stop() {
        MOTOR.stop();
    }

    @Override
    protected void refreshInputs(KickerInputsAutoLogged inputs) {
        inputs.doesSeeNote = true; // We assume a NOTE is always inside the shooter for simulation
        inputs.voltage = MOTOR.getVoltage();
    }
}
