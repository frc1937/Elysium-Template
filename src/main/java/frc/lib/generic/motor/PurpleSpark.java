package frc.lib.generic.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

public class PurpleSpark extends CANSparkBase implements Motor {
    private final MotorProperties.SparkType model;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    private MotorConfiguration currentConfiguration;

    private double closedLoopTarget;
    private Feedforward feedforward;

    private int slotToUse = 0;

    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();

    private final Supplier<TrapezoidProfile.State> currentStateSupplier = () -> new TrapezoidProfile.State(getSystemPosition(), getSystemVelocity());
    private Function<TrapezoidProfile.State, Double> feedforwardSupplier = (targetState) -> 0.0;

    private PIDController feedback;
    private TrapezoidProfile motionProfile;

    public PurpleSpark(int deviceId, MotorProperties.SparkType sparkType) {
        super(deviceId, MotorType.kBrushless, sparkType == MotorProperties.SparkType.MAX ? SparkModel.SparkMax : SparkModel.SparkFlex);
        model = sparkType;

        optimizeBusUsage();

        encoder = this.getEncoder();
        controller = super.getPIDController();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setGoal(mode, output);

        //todo: Implement custom FF

        switch (mode) {
            case PERCENTAGE_OUTPUT -> controller.setReference(output, ControlType.kDutyCycle);

            case VELOCITY, POSITION -> setModeBasedTarget(mode);

            case VOLTAGE -> controller.setReference(output, ControlType.kVoltage, slotToUse);
            case CURRENT -> controller.setReference(output, ControlType.kCurrent, slotToUse);
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? IdleMode.kCoast : IdleMode.kBrake);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public int getDeviceID() {
        return getDeviceId();
    }

    @Override
    public MotorProperties.Slot getCurrentSlot() {
        return getSlot(slotToUse, currentConfiguration);
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void resetSlot(MotorProperties.Slot slot, int slotNumber) {
        switch (slotNumber) {
            case 0 -> currentConfiguration.slot0 = slot;
            case 1 -> currentConfiguration.slot1 = slot;
            case 2 -> currentConfiguration.slot2 = slot;
        }

        configure(currentConfiguration);
    }

    @Override
    public double getMotorPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getMotorVelocity() {
        return encoder.getVelocity() / 60;
    }

    @Override
    public double getCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public double getTemperature() {
        return getMotorTemperature();
    }

    @Override
    public double getSystemPosition() {
        Logger.recordOutput("ArmMotor/Position", encoder.getPosition());
        Logger.recordOutput("ArmMotor/Position + GEARING APPLIED", encoder.getPosition() * currentConfiguration.gearRatio);

        return  encoder.getPosition();
    }

    @Override
    public double getSystemVelocity() {
        return encoder.getVelocity() / (60 * currentConfiguration.gearRatio);
    }

    @Override
    public double getClosedLoopTarget() {
        return closedLoopTarget;
    }

    @Override
    public double getVoltage() {
        return super.getBusVoltage() * getAppliedOutput();
    }


    @Override
    public void setFollowerOf(int masterPort) {
        super.follow(new GenericSpark(masterPort, model));
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">REV DOCS</a>
     */
    @Override
    public void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz) {
        int ms = (int) (1000 / updateFrequencyHz);

        switch (signalType) {
            case VELOCITY, CURRENT -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
            case POSITION -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, ms);
        }
    }

    @Override
    public void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes) {
        for (Properties.SignalType signalType : signalTypes) {
            setSignalUpdateFrequency(signalType, updateFrequencyHz);
        }
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public void refreshStatusSignals(Properties.SignalType... signalTypes) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public TalonFXSimState getSimulationState() {
        throw new UnsupportedOperationException("Not supported. Use GenericTalonFX");
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        super.restoreFactoryDefaults();

        super.setIdleMode(configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? IdleMode.kBrake : IdleMode.kCoast);
        super.setInverted(configuration.inverted);

        super.enableVoltageCompensation(12);

        if (configuration.statorCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureProfile(configuration);

        configurePID(configuration);
        configureFeedForward(configuration);

        return super.burnFlash() == REVLibError.kOk;
    }

    private void configureFeedForward(MotorConfiguration configuration) {
        MotorProperties.Slot currentSlot = configuration.slot0;

        switch (slotToUse) {
            case 1 -> currentSlot = configuration.slot1;
            case 2 -> currentSlot = configuration.slot2;
        }

        if (currentSlot.gravityType() == null) {
            feedforward = new Feedforward(Properties.FeedforwardType.SIMPLE,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA()
            );
        }

        if (currentSlot.gravityType() == GravityTypeValue.Arm_Cosine) {
            feedforward = new Feedforward(Properties.FeedforwardType.ARM,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA(),
                    currentSlot.kG()
            );
        }

        if (currentSlot.gravityType() == GravityTypeValue.Elevator_Static) {
            feedforward = new Feedforward(Properties.FeedforwardType.ELEVATOR,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA(),
                    currentSlot.kG()
            );
        }

        feedforwardSupplier = (targetState) ->
                feedforward.calculate(targetState.position, targetState.velocity, 0);
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profiledMaxVelocity == 0 || configuration.profiledTargetAcceleration == 0)
            return;

        final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                configuration.profiledMaxVelocity,
                configuration.profiledTargetAcceleration
        );

        motionProfile = new TrapezoidProfile(constraints);
    }

    private void configurePID(MotorConfiguration configuration) {
        feedback = new PIDController(configuration.slot0.kP(), configuration.slot0.kI(), configuration.slot0.kD());

        if (configuration.closedLoopContinuousWrap)
            feedback.enableContinuousInput(-0.5, +0.5);

        slotToUse = configuration.slotToUse;
    }

    private void setModeBasedTarget(MotorProperties.ControlMode controlMode) {
        final double feedforwardOutput, feedbackOutput;

        if (motionProfile == null) {
            feedforwardOutput = feedforwardSupplier.apply(goalState);
            feedbackOutput = getModeBasedFeedback(controlMode, goalState);
        } else {
            final TrapezoidProfile.State targetProfiledState = motionProfile.calculate(0.02, currentStateSupplier.get(), goalState);

            feedforwardOutput = feedforwardSupplier.apply(targetProfiledState);
            feedbackOutput = getModeBasedFeedback(controlMode, targetProfiledState);

            Logger.recordOutput("Motor/ProfiledState position", targetProfiledState.position);
            Logger.recordOutput("Motor/ProfiledState velocity", targetProfiledState.velocity);
        }



        Logger.recordOutput("Motor/ProfiledState feedforwardOutput", feedforwardOutput);
        Logger.recordOutput("Motor/ProfiledState feedbackOutput", feedbackOutput);

        Logger.recordOutput("Motor/ProfiledState voltageOutput", feedbackOutput + feedforwardOutput);

        controller.setReference(
                feedforwardOutput + feedbackOutput,
                ControlType.kVoltage
        );
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 10000);
    }

    @Override
    public RelativeEncoder getEncoder() {
        switch (model) {
            case MAX -> {
                return getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
            }

            case FLEX -> {
                return getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
            }
        }

        return null;
    }

    private void setGoal(MotorProperties.ControlMode controlMode, double output) {
        final TrapezoidProfile.State newState = setModeBasedState(controlMode, output);

        if (!Objects.equals(goalState, newState)) {
            feedback.reset();
            goalState = newState;

            DriverStation.reportError("[PurpleSpark] goal has changed", false);
        }
    }

    private TrapezoidProfile.State setModeBasedState(MotorProperties.ControlMode controlMode, double output) {
        final TrapezoidProfile.State newState;

        switch (controlMode) {
            case POSITION -> newState = new TrapezoidProfile.State(output, 0.0);
            case VELOCITY -> newState = new TrapezoidProfile.State(0, output);

            default -> throw new IllegalStateException("Unexpected value: " + controlMode);
        }

        return newState;
    }

    private double getModeBasedFeedback(MotorProperties.ControlMode controlMode, TrapezoidProfile.State targetState) {
        final double feedbackOutput;

        switch (controlMode) {
            case POSITION -> feedbackOutput = feedback.calculate(getSystemPosition(), targetState.position);
            case VELOCITY -> feedbackOutput = feedback.calculate(getSystemVelocity(), targetState.velocity);
            default -> throw new IllegalStateException("Unexpected value: " + controlMode);
        }

        return feedbackOutput;
    }}
