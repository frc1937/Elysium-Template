package frc.lib.generic.hardware.motor.hardware.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

public class GenericSparkMax extends GenericSparkBase {
    private CANSparkBase spark;
    private RelativeEncoder encoder;
    private SparkPIDController sparkController;

    private PIDController feedback;

    private double lastProfileCalculationTimestamp;
    private TrapezoidProfile.State previousSetpoint;

    public GenericSparkMax(String name, int deviceId) {
        super(name, deviceId);
    }

    @Override
    protected double getLastProfileCalculationTimestamp() {
        return lastProfileCalculationTimestamp;
    }

    @Override
    protected void setPreviousSetpoint(TrapezoidProfile.State previousSetpoint) {
        this.previousSetpoint = previousSetpoint;
    }

    @Override
    protected CANSparkBase getSpark() {
        if (spark == null) spark = new CANSparkMax(getDeviceID(), CANSparkLowLevel.MotorType.kBrushless);
        return spark;
    }

    @Override
    protected RelativeEncoder getEncoder() {
        if (encoder == null) encoder = spark.getEncoder();
        return encoder;
    }

    @Override
    protected SparkPIDController getSparkController() {
        if (sparkController == null) sparkController = spark.getPIDController();
        return sparkController;
    }

    @Override
    protected void refreshExtras() {

    }


    @Override
    protected void setNewGoalExtras() {
        feedback.reset();
    }

    @Override
    protected void configureExtras(MotorConfiguration configuration) {

    }

    protected void configurePID(MotorConfiguration configuration) {
        feedback = new PIDController(configuration.slot0.kP(), configuration.slot0.kI(), configuration.slot0.kD());

        if (configuration.slotToUse == 1)
            feedback = new PIDController(configuration.slot1.kP(), configuration.slot1.kI(), configuration.slot1.kD());
        if (configuration.slotToUse == 2)
            feedback = new PIDController(configuration.slot2.kP(), configuration.slot2.kI(), configuration.slot2.kD());

        if (configuration.closedLoopContinuousWrap)
            feedback.enableContinuousInput(-0.5, 0.5);
    }

    protected void handleSmoothMotion(MotorProperties.ControlMode controlMode, TrapezoidProfile.State goalState, TrapezoidProfile motionProfile, Feedforward.Type feedforward, int slotToUse) {
        if (goalState == null) return;

        double feedbackOutput = 0, feedforwardOutput = 0, acceleration;

        if (motionProfile != null) {
            final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

            if (controlMode ==MotorProperties.ControlMode.POSITION) {
                acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;

                feedforwardOutput = feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity, acceleration);
                feedbackOutput = feedback.calculate(getEffectivePosition(), currentSetpoint.position);
            }

            if (controlMode == MotorProperties.ControlMode.VELOCITY) {
                feedforwardOutput = feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity);
                feedbackOutput = this.feedback.calculate(getEffectiveVelocity(), currentSetpoint.position);
            }

            previousSetpoint = currentSetpoint;
            lastProfileCalculationTimestamp = Logger.getTimestamp();
        } else {
            feedbackOutput = getModeBasedFeedback(controlMode, goalState);
            feedforwardOutput = feedforward.calculate(goalState.position, goalState.velocity, 0);
        }

        sparkController.setReference(feedforwardOutput + feedbackOutput, CANSparkBase.ControlType.kVoltage);
    }

    private double getModeBasedFeedback(MotorProperties.ControlMode mode, TrapezoidProfile.State goal) {
        if (mode == MotorProperties.ControlMode.POSITION) {
            return feedback.calculate(getEffectivePosition(), goal.position);
        } else if (mode == MotorProperties.ControlMode.VELOCITY) {
            return feedback.calculate(getEffectiveVelocity(), goal.velocity);
        }

        return 0;
    }
}
