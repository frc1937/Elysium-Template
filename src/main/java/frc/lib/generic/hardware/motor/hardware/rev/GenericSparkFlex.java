package frc.lib.generic.hardware.motor.hardware.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.scurve.InputParameter;
import frc.lib.scurve.OutputParameter;
import frc.lib.scurve.UpdateResult;
import org.littletonrobotics.junction.Logger;

public class GenericSparkFlex extends GenericSparkBase {
    private CANSparkBase spark;
    private RelativeEncoder encoder;
    private SparkPIDController sparkController;

    private InputParameter scurveInputs;
    private OutputParameter scurveOutput = new OutputParameter();

    private double lastProfileCalculationTimestamp;
    private TrapezoidProfile.State previousSetpoint;

    private final Timer timer = new Timer();

    public GenericSparkFlex(String name, int deviceId) {
        super(name, deviceId);

        timer.start();
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
        if (spark == null) spark = new CANSparkFlex(getDeviceID(), CANSparkLowLevel.MotorType.kBrushless);
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
        if (timer.advanceIfElapsed(4)) {
            encoder.setPosition(getEffectivePosition());
        }
    }

    @Override
    protected void setNewGoalExtras() {
    }

    @Override
    protected void configureExtras(MotorConfiguration configuration) {
        encoder.setPositionConversionFactor(1 / configuration.gearRatio);
        encoder.setVelocityConversionFactor(1/ (configuration.gearRatio));
        encoder.setPosition(getEffectivePosition());
    }

    protected void configurePID(MotorConfiguration configuration) {
        sparkController.setP(configuration.slot0.kP(), 0);
        sparkController.setI(configuration.slot0.kI(), 0);
        sparkController.setD(configuration.slot0.kD(), 0);

        sparkController.setP(configuration.slot1.kP(), 1);
        sparkController.setI(configuration.slot1.kI(), 1);
        sparkController.setD(configuration.slot1.kD(), 1);

        sparkController.setP(configuration.slot2.kP(), 2);
        sparkController.setI(configuration.slot2.kI(), 2);
        sparkController.setD(configuration.slot2.kD(), 2);

        sparkController.setPositionPIDWrappingEnabled(configuration.closedLoopContinuousWrap);
        sparkController.setSmartMotionAllowedClosedLoopError(configuration.closedLoopTolerance, configuration.slotToUse);
        //check if works, and if theres a default
    }

    protected void handleSmoothMotion(SparkCommon.MotionType motionType,
                                      TrapezoidProfile.State goalState, TrapezoidProfile motionProfile,
                                      Feedforward feedforward, int slotToUse) {
        if (goalState == null) return;

        double feedforwardOutput, acceleration;

        switch (motionType) {
            case POSITION_PID -> {
                sparkController.setReference(goalState.position,
                        CANSparkBase.ControlType.kPosition, slotToUse,
                        0,
                        SparkPIDController.ArbFFUnits.kVoltage);
            }

            case VELOCITY_PID_FF -> {
                sparkController.setReference(goalState.position * 60,
                        CANSparkBase.ControlType.kVelocity, slotToUse,
                        feedforward.calculate(goalState.position, goalState.velocity, 0),
                        SparkPIDController.ArbFFUnits.kVoltage);
            }

            case POSITION_TRAPEZOIDAL -> {
                final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

                acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;
                feedforwardOutput = feedforward.calculate(getEffectivePosition(), currentSetpoint.velocity, acceleration);

                sparkController.setReference(currentSetpoint.position,
                        CANSparkBase.ControlType.kPosition,
                        slotToUse, feedforwardOutput,
                        SparkPIDController.ArbFFUnits.kVoltage);

                previousSetpoint = currentSetpoint;
                lastProfileCalculationTimestamp = Logger.getRealTimestamp();
            }

            case VELOCITY_TRAPEZOIDAL -> {
                final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

                feedforwardOutput = feedforward.calculate(0, currentSetpoint.position, currentSetpoint.velocity);

                sparkController.setReference(currentSetpoint.position * 60,
                        CANSparkBase.ControlType.kVelocity,
                        slotToUse, feedforwardOutput,
                        SparkPIDController.ArbFFUnits.kVoltage);


                previousSetpoint = currentSetpoint;
                lastProfileCalculationTimestamp = Logger.getRealTimestamp();
            }

            case POSITION_S_CURVE -> {
                final UpdateResult result = getSCurveGenerator().update(scurveInputs, scurveOutput);

                scurveInputs = result.input_parameter;
                scurveOutput = result.output_parameter;

                feedforwardOutput = feedforward.calculate(getEffectivePosition(), scurveOutput.new_velocity, scurveOutput.new_acceleration);

                sparkController.setReference(scurveOutput.new_position,
                        CANSparkBase.ControlType.kPosition,
                        slotToUse, feedforwardOutput,
                        SparkPIDController.ArbFFUnits.kVoltage);

                lastProfileCalculationTimestamp = Logger.getRealTimestamp();
            }
        }
    }

    protected void setSCurveInputs(InputParameter scurveInputs) {
        this.scurveInputs = scurveInputs;
    }

    protected void setSCurveOutputs(OutputParameter scurveOutput) {
        this.scurveOutput = scurveOutput;
    }
}
