package frc.robot.subsystems.intake.real;

import frc.lib.generic.motor.MotorProperties;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;

import static frc.robot.subsystems.intake.real.RealIntakeConstants.MOTOR;

public class RealIntake extends IntakeIO {

    @Override
    public void setPercentageOutput(double percentageOutput) {
        MOTOR.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentageOutput);
    }

    @Override
    public void stop() {
        MOTOR.stopMotor();
    }

    @Override
    protected void refreshInputs(IntakeInputsAutoLogged intakeInputs) {
        intakeInputs.voltage = MOTOR.getVoltage();
    }
}
