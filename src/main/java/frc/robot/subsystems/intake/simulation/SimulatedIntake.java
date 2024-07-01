package frc.robot.subsystems.intake.simulation;

import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;

import static frc.robot.subsystems.intake.simulation.SimulatedIntakeConstants.MOTOR;

public class SimulatedIntake extends IntakeIO {
    @Override
    public void setPercentageOutput(double percentage) {
        MOTOR.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentage);
    }

    @Override
    public void stop() {
        MOTOR.stop();
    }

    @Override
    protected void refreshInputs(IntakeInputsAutoLogged intakeInputs) {
        intakeInputs.voltage = MOTOR.getVoltage();
    }
}
