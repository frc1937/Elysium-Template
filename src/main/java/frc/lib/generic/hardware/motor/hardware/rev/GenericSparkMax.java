package frc.lib.generic.hardware.motor.hardware.rev;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.PID;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.scurve.InputParameter;
import frc.lib.scurve.OutputParameter;
import frc.lib.scurve.UpdateResult;
import org.littletonrobotics.junction.Logger;

public class GenericSparkMax extends GenericSparkBase {
    private CANSparkBase spark;
    private RelativeEncoder encoder;
    private SparkPIDController sparkController;

    private PID feedback;

    private InputParameter scurveInputs;
    private OutputParameter scurveOutput = new OutputParameter();

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
        feedback = new PID(configuration.slot0.kP(), configuration.slot0.kI(), configuration.slot0.kD(), configuration.slot0.kS());

        if (configuration.slotToUse == 1)
            feedback = new PID(configuration.slot1.kP(), configuration.slot1.kI(), configuration.slot1.kD(), configuration.slot1.kS());
        if (configuration.slotToUse == 2)
            feedback = new PID(configuration.slot2.kP(), configuration.slot2.kI(), configuration.slot2.kD(), configuration.slot2.kS());

        if (configuration.closedLoopContinuousWrap)
            feedback.enableContinuousInput(-0.5, 0.5);
    }

    protected void handleSmoothMotion(SparkCommon.MotionType motionType, TrapezoidProfile.State goalState, TrapezoidProfile motionProfile,
                                      Feedforward feedforward, int slotToUse) {
        if (goalState == null) return;

        double feedbackOutput = 0, feedforwardOutput = 0, acceleration;

        switch (motionType) {
            case POSITION_TRAPEZOIDAL -> {
                final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

                acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;

                feedforwardOutput = feedforward.calculate(getEffectivePosition(), currentSetpoint.velocity, acceleration);
                feedbackOutput = feedback.calculate(getEffectivePosition(), currentSetpoint.position);

                previousSetpoint = currentSetpoint;
                lastProfileCalculationTimestamp = Logger.getTimestamp();
            }

            case VELOCITY_TRAPEZOIDAL -> {
                final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

                feedforwardOutput = feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity);
                feedbackOutput = this.feedback.calculate(getEffectiveVelocity(), currentSetpoint.position);

                previousSetpoint = currentSetpoint;
                lastProfileCalculationTimestamp = Logger.getTimestamp();
            }

            case VELOCITY_PID_FF -> {
                feedforwardOutput = feedforward.calculate(goalState.position, goalState.velocity);
                feedbackOutput = this.feedback.calculate(getEffectiveVelocity(), goalState.position);
            }

            case POSITION_PID -> feedbackOutput = this.feedback.calculate(getEffectivePosition(), goalState.position);

            case POSITION_S_CURVE -> {
                final UpdateResult result = getSCurveGenerator().update(scurveInputs, scurveOutput);

                scurveInputs = result.input_parameter;
                scurveOutput = result.output_parameter;

                feedforwardOutput = feedforward.calculate(getEffectivePosition(), scurveOutput.new_velocity, scurveOutput.new_acceleration);
                feedbackOutput = feedback.calculate(getEffectivePosition(), scurveOutput.new_position);
            }
        }

        sparkController.setReference(feedforwardOutput + feedbackOutput, CANSparkBase.ControlType.kVoltage);
    }

    @Override
    protected void setSCurveInputs(InputParameter scurveInputs) {
        this.scurveInputs = scurveInputs;
    }

    @Override
    protected void setSCurveOutputs(OutputParameter outputParameter) {
        this.scurveOutput = outputParameter;
    }
}
