package frc.robot.subsystems.intake.real;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;

import static frc.robot.subsystems.intake.real.RealIntakeConstants.MOTOR;

public class RealIntake extends IntakeIO {

    @Override
    public void setPercentageOutput(double percentageOutput) {
        MOTOR.set(ControlMode.PercentOutput, percentageOutput);
    }

    @Override
    public void stop() {
        MOTOR.stopMotor();
    }

    @Override
    protected void refreshInputs(IntakeInputsAutoLogged intakeInputs) {
        intakeInputs.voltage = MOTOR.getMotorOutputVoltage() * 12;
    }
}
