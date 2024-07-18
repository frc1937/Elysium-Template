package frc.lib.generic.motor.hardware;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorInputsAutoLogged;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.MotorSignal;
import frc.lib.math.Conversions;
import frc.robot.poseestimation.poseestimator.OdometryThread;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class GenericSpark extends Motor {
    private static final double useBuiltinFeedforwardNumber = 69420;

    private final CANSparkBase spark;
    private final RelativeEncoder encoder;

    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();
    private final Queue<Double> timestampQueue;

    private double closedLoopTarget;

    private MotorConfiguration currentConfiguration;
    private Feedforward feedforward;

    private int slotToUse = 0;
    private double conversionFactor = 1;

    private PIDController feedback;

    private DoubleSupplier positionSupplier, velocitySupplier;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State previousSetpoint, goalState;

    public GenericSpark(String name, int deviceId, MotorProperties.SparkType sparkType) {
        super(name);

        if (sparkType == MotorProperties.SparkType.FLEX)
            spark = new CANSparkFlex(deviceId, CANSparkFlex.MotorType.kBrushless);
        else spark = new CANSparkMax(deviceId, CANSparkMax.MotorType.kBrushless);

        optimizeBusUsage();

        encoder = spark.getEncoder();
        timestampQueue = OdometryThread.getInstance().getTimestampQueue();
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
            case PERCENTAGE_OUTPUT -> spark.set(output);

            case POSITION, VELOCITY -> handleSmoothMotion(mode, feedforward);

            case VOLTAGE -> spark.setVoltage(output);
            case CURRENT -> spark.getPIDController().setReference(output, CANSparkBase.ControlType.kCurrent, slotToUse);
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        spark.setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public void stopMotor() {
        spark.stopMotor();
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public int getDeviceID() {
        return spark.getDeviceId();
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
    public void setFollowerOf(String name, int masterPort) {
        spark.follow(new CANSparkMax(masterPort, CANSparkMax.MotorType.kBrushless));
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
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
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        spark.restoreFactoryDefaults();

        setIdleMode(configuration.idleMode);
        spark.setInverted(configuration.inverted);

        spark.enableVoltageCompensation(12);

        conversionFactor = (1.0 / configuration.gearRatio);

        if (configuration.statorCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureProfile(configuration);
        configurePID(configuration);

        configureFeedForward();

        return spark.burnFlash() == REVLibError.kOk;
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

        if (slotToUse == 1)
            feedback = new PIDController(configuration.slot1.kP(), configuration.slot1.kI(), configuration.slot1.kD());
        if (slotToUse == 2)
            feedback = new PIDController(configuration.slot2.kP(), configuration.slot2.kI(), configuration.slot2.kD());

        if (configuration.closedLoopContinuousWrap)
            feedback.enableContinuousInput(-0.5, 0.5);
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 10000);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, 10000);
    }

    private void handleSmoothMotion(MotorProperties.ControlMode controlMode, double feedforward) {
        double feedforwardOutput = 0, feedbackOutput, acceleration = 0;

        if (motionProfile == null) {
            feedbackOutput = getModeBasedFeedback(controlMode, goalState);
            //todo: acceleration

            if (controlMode == MotorProperties.ControlMode.VELOCITY)
                feedforwardOutput = getFeedforwardOutput(goalState, acceleration);
            if (controlMode == MotorProperties.ControlMode.POSITION)
                feedforwardOutput = getFeedforwardOutput(goalState, acceleration);
        } else {
            final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

            acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;

            feedforwardOutput = getFeedforwardOutput(currentSetpoint, acceleration);
            feedbackOutput = getModeBasedFeedback(controlMode, currentSetpoint);

            previousSetpoint = currentSetpoint;
        }

        if (feedforward != useBuiltinFeedforwardNumber)
            feedforwardOutput = feedforward;

        spark.setVoltage(feedforwardOutput + feedbackOutput);
    }

    private void setGoal(MotorProperties.ControlMode controlMode, double output) {
        TrapezoidProfile.State stateFromOutput = null;

        if (controlMode == MotorProperties.ControlMode.POSITION)
            stateFromOutput = new TrapezoidProfile.State(output, 0);
        if (controlMode == MotorProperties.ControlMode.VELOCITY)
            stateFromOutput = new TrapezoidProfile.State(0, output);

        if (stateFromOutput != null && goalState == null || !goalState.equals(stateFromOutput)) {
            feedback.reset();

            previousSetpoint = new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity());
            goalState = stateFromOutput;
        }
    }

    private double getModeBasedFeedback(MotorProperties.ControlMode mode, TrapezoidProfile.State goal) {
        if (mode != MotorProperties.ControlMode.POSITION && mode != MotorProperties.ControlMode.VELOCITY) return 0;

        if (mode == MotorProperties.ControlMode.POSITION) {
            return feedback.calculate(getEffectivePosition(), goal.position);
        }

        return feedback.calculate(getEffectiveVelocity(), goal.velocity);
    }

    private double getFeedforwardOutput(TrapezoidProfile.State goal, double acceleration) {
        if (getCurrentSlot().gravityType() == null || getCurrentSlot().gravityType() == GravityTypeValue.Elevator_Static)
            return feedforward.calculate(goal.velocity, acceleration);

        return feedforward.calculate(goal.position, goal.velocity, acceleration);
    }

    private double getEffectivePosition() {
        return positionSupplier == null ? getSystemPositionPrivate() : positionSupplier.getAsDouble();
    }

    private double getEffectiveVelocity() {
        return velocitySupplier == null ? getSystemVelocityPrivate() : velocitySupplier.getAsDouble();
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces">REV DOCS</a>
     */
    private void setupSignalUpdates(MotorSignal signal) {
        int ms = (int) (1000 / signal.getUpdateRate());

        switch (signal.getType()) {
            case VELOCITY, CURRENT, TEMPERATURE ->
                    spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, ms);
            case POSITION -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, ms);
        }

        if (!signal.useFasterThread()) return;

        switch (signal.getType()) {
            case POSITION ->
                    signalQueueList.put("position", OdometryThread.getInstance().registerSignal(this::getSystemPositionPrivate));
            case VELOCITY ->
                    signalQueueList.put("velocity", OdometryThread.getInstance().registerSignal(this::getSystemVelocityPrivate));

            case CURRENT ->
                    signalQueueList.put("current", OdometryThread.getInstance().registerSignal(spark::getOutputCurrent));
            case VOLTAGE ->
                    signalQueueList.put("voltage", OdometryThread.getInstance().registerSignal(this::getVoltagePrivate));

            case TEMPERATURE ->
                    signalQueueList.put("temperature", OdometryThread.getInstance().registerSignal(spark::getMotorTemperature));
            case CLOSED_LOOP_TARGET ->
                    signalQueueList.put("target", OdometryThread.getInstance().registerSignal(() -> closedLoopTarget));
        }
    }

    @Override
    protected void refreshInputs(MotorInputsAutoLogged inputs) {
        if (spark == null) return;

        inputs.systemPosition = getEffectivePosition();
        inputs.systemVelocity = getEffectiveVelocity();

        inputs.voltage = getVoltagePrivate();
        inputs.current = spark.getOutputCurrent();
        inputs.temperature = spark.getMotorTemperature();

        inputs.target = closedLoopTarget;

        MotorUtilities.handleThreadedInputs(inputs, signalQueueList, timestampQueue);
    }

    private double getVoltagePrivate() {
        return spark.getBusVoltage() * spark.getAppliedOutput();
    }

    private double getSystemPositionPrivate() {
        return encoder.getPosition() * conversionFactor;
    }

    private double getSystemVelocityPrivate() {
        return (encoder.getVelocity() / Conversions.SEC_PER_MIN) * conversionFactor;
    }
}
