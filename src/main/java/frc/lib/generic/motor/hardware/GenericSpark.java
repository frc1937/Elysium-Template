package frc.lib.generic.motor.hardware;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.MotorSignal;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class GenericSpark extends CANSparkBase implements Motor {
    private static final double useBuiltinFeedforwardNumber = 69420;

    private final MotorProperties.SparkType model;
    private final RelativeEncoder encoder;

    private final String name;

    private double closedLoopTarget;

    private MotorConfiguration currentConfiguration;
    private Feedforward feedforward;

    private int slotToUse = 0;
    private double conversionFactor = 1;

    private PIDController feedback;

    private DoubleSupplier positionSupplier, velocitySupplier;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State previousSetpoint, goalState;

    private double previousTimestamp = Logger.getTimestamp();

    public GenericSpark(String name, int deviceId, MotorProperties.SparkType sparkType) {
        super(deviceId, MotorType.kBrushless, sparkType == MotorProperties.SparkType.MAX ? SparkModel.SparkMax : SparkModel.SparkFlex);

        model = sparkType;
        this.name = name;

        optimizeBusUsage();

        encoder = this.getEncoder();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, useBuiltinFeedforwardNumber);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setGoal(mode, output);

        switch (mode) {
            case PERCENTAGE_OUTPUT -> set(output);

            case POSITION, VELOCITY -> handleSmoothMotion(mode, feedforward);

            case VOLTAGE -> setVoltage(output);
            case CURRENT -> super.getPIDController().setReference(output, ControlType.kCurrent, slotToUse);
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
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) {
        this.positionSupplier = positionSupplier;
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) {
        this.velocitySupplier = velocitySupplier;
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
        return encoder.getPosition() * conversionFactor;
    }

    @Override
    public double getSystemVelocity() {
        return (encoder.getVelocity() / Conversions.SEC_PER_MIN) * conversionFactor;
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
    public void setFollowerOf(String name, int masterPort) {
        super.follow(new GenericSpark(name, masterPort, model));
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">REV DOCS</a>
     */
    @Override
    public void setupSignalUpdates(MotorSignal signal) {
        int ms = (int) (1000 / signal.getUpdateRate());

        switch (signal.getType()) {
            case VELOCITY, CURRENT -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
            case POSITION -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, ms);
        }
    }

    @Override
    public void setupSignalsUpdates(MotorSignal... signals) {
        for (MotorSignal signalType : signals) {
            setupSignalUpdates(signalType);
        }
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(MotorSignal signal) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public void refreshStatusSignals(MotorSignal... signals) {
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

        conversionFactor = (1.0 / configuration.gearRatio);

        if (configuration.statorCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureProfile(configuration);
        configurePID(configuration);

        configureFeedForward();

        return super.burnFlash() == REVLibError.kOk;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profiledMaxVelocity == 0 || configuration.profiledTargetAcceleration == 0) return;

        final TrapezoidProfile.Constraints motionConstraints = new TrapezoidProfile.Constraints(
                configuration.profiledMaxVelocity,
                configuration.profiledTargetAcceleration
        );

        motionProfile = new TrapezoidProfile(motionConstraints);
    }

    private void configureFeedForward() {
        MotorProperties.Slot currentSlot = getCurrentSlot();

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
    }

    private void configurePID(MotorConfiguration configuration) {
        slotToUse = configuration.slotToUse;

        feedback = new PIDController(configuration.slot0.kP(), configuration.slot0.kI(), configuration.slot0.kD());

        if (slotToUse == 1) feedback = new PIDController(configuration.slot1.kP(), configuration.slot1.kI(), configuration.slot1.kD());
        if (slotToUse == 2) feedback = new PIDController(configuration.slot2.kP(), configuration.slot2.kI(), configuration.slot2.kD());

        if (configuration.closedLoopContinuousWrap)
            feedback.enableContinuousInput(-0.5, 0.5);
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

    private void handleSmoothMotion(MotorProperties.ControlMode controlMode, double feedforward) {
        final double timeDifference = ((Logger.getTimestamp() - previousTimestamp) / 1000000);

        double feedforwardOutput = 0, feedbackOutput, acceleration = 0;

        if (motionProfile == null) {
            feedbackOutput = getModeBasedFeedback(controlMode, goalState);
            //todo: acceleration

            if (controlMode == MotorProperties.ControlMode.VELOCITY) feedforwardOutput = getFeedforwardOutput(goalState, acceleration);
            if (controlMode == MotorProperties.ControlMode.POSITION) feedforwardOutput = getFeedforwardOutput(goalState, acceleration);
        } else {
            final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(timeDifference, previousSetpoint, goalState);

            acceleration = currentSetpoint.velocity - previousSetpoint.velocity / timeDifference;

            feedforwardOutput = getFeedforwardOutput(currentSetpoint, acceleration);
            feedbackOutput = getModeBasedFeedback(controlMode, currentSetpoint);

            previousSetpoint = currentSetpoint;

            Logger.recordOutput("ProfiledPOSITION", currentSetpoint.position * 360);
            Logger.recordOutput("ProfiledVELOCITY", currentSetpoint.velocity * 360);
        }

        if (feedforward != useBuiltinFeedforwardNumber)
            feedforwardOutput = feedforward;

        super.setVoltage(feedforwardOutput + feedbackOutput);

        previousTimestamp = Logger.getTimestamp();

        Logger.recordOutput("Profiled CURRENT POSITION", getEffectivePosition() * 360);
        Logger.recordOutput("Profiled CURRENT VELOCITY", getEffectiveVelocity() * 360);

        Logger.recordOutput("FeeFF", feedforwardOutput);
        Logger.recordOutput("FeeFB", feedbackOutput);
        Logger.recordOutput("FeeVOLT", feedforwardOutput + feedbackOutput);
    }


    private void setGoal(MotorProperties.ControlMode controlMode, double output) {
        TrapezoidProfile.State stateFromOutput = null;

        if (controlMode == MotorProperties.ControlMode.POSITION) stateFromOutput = new TrapezoidProfile.State(output, 0);
        if (controlMode == MotorProperties.ControlMode.VELOCITY) stateFromOutput = new TrapezoidProfile.State(0, output);

        if (stateFromOutput != null && goalState == null || !goalState.equals(stateFromOutput)) {
            feedback.reset();

            previousSetpoint = new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity());
            goalState = stateFromOutput;

            DriverStation.reportError("[PurpleSpark] goal has changed", false);
        }
    }

    private double getModeBasedFeedback(MotorProperties.ControlMode mode, TrapezoidProfile.State goal) {
        if (mode == null) return 0;

        if (mode == MotorProperties.ControlMode.POSITION) {

            return feedback.calculate(getEffectivePosition(), goal.position);
        }

        if (mode == MotorProperties.ControlMode.VELOCITY) {
            return feedback.calculate(getEffectiveVelocity(), goal.velocity);
        }

        return 0;
    }

    private double getFeedforwardOutput(TrapezoidProfile.State goal, double acceleration) {
        if (getCurrentSlot().gravityType() == null || getCurrentSlot().gravityType() == GravityTypeValue.Elevator_Static)
            return feedforward.calculate(goal.velocity, acceleration);

        return feedforward.calculate(goal.position, goal.velocity, acceleration);
    }

    private double getEffectivePosition() {
        return positionSupplier == null ? getSystemPosition() : positionSupplier.getAsDouble();
    }

    private double getEffectiveVelocity() {
        return velocitySupplier == null ? getSystemVelocity() : velocitySupplier.getAsDouble();
    }
}
