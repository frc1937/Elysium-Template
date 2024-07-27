package frc.lib.generic.hardware.motor.hardware;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.math.Conversions;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;

public class GenericSpark extends Motor {
    private static final double USE_BUILTIN_FEEDFORWARD_NUMBER = 69420;

    private final CANSparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private double closedLoopTarget;

    private MotorConfiguration currentConfiguration;
    private Feedforward.Type feedforward;
    private PIDController feedback;

    private int slotToUse = 0;
    private double conversionFactor = 1;

    private DoubleSupplier externalPositionSupplier, externalVelocitySupplier;

    private TrapezoidProfile positionMotionProfile, velocityMotionProfile;
    private TrapezoidProfile.State previousSetpoint, goalState;

    private double previousVelocity = 0;

    public GenericSpark(String name, int deviceId, MotorProperties.SparkType sparkType) {
        super(name);

        spark = sparkType.sparkCreator.apply(deviceId);

        encoder = spark.getEncoder();
        controller = spark.getPIDController();

        optimizeBusUsage();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, USE_BUILTIN_FEEDFORWARD_NUMBER);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setGoal(mode, output);



        switch (mode) {
            case PERCENTAGE_OUTPUT -> controller.setReference(output, CANSparkBase.ControlType.kDutyCycle);

            case POSITION, VELOCITY -> handleSmoothMotion(mode, feedforward);

            case VOLTAGE -> controller.setReference(output, CANSparkBase.ControlType.kVoltage, slotToUse, 0);
            case CURRENT -> controller.setReference(output, CANSparkBase.ControlType.kCurrent, slotToUse, 0);
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
        this.externalPositionSupplier = positionSupplier;
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) {
        this.externalVelocitySupplier = velocitySupplier;
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void setFollowerOf(String name, int masterPort) {
        spark.follow(new CANSparkMax(masterPort, CANSparkLowLevel.MotorType.kBrushless));
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
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
        if (configuration.profiledMaxVelocity != 0 && configuration.profiledTargetAcceleration == 0) {
            final TrapezoidProfile.Constraints positionMotionConstraints = new TrapezoidProfile.Constraints(
                    configuration.profiledMaxVelocity,
                    configuration.profiledTargetAcceleration
            );

            positionMotionProfile = new TrapezoidProfile(positionMotionConstraints);
        }

        if (configuration.profiledTargetAcceleration != 0 && configuration.profiledJerk != 0) {
            final TrapezoidProfile.Constraints velocityMotionConstraints = new TrapezoidProfile.Constraints(
                    configuration.profiledTargetAcceleration,
                    configuration.profiledJerk
            );

            velocityMotionProfile = new TrapezoidProfile(velocityMotionConstraints);
        }
    }

    private void configureFeedForward() {
        MotorProperties.Slot currentSlot = getCurrentSlot();

        if (currentSlot.gravityType() == null)
            feedforward = Feedforward.Type.SIMPLE;

        if (currentSlot.gravityType() == GravityTypeValue.Arm_Cosine)
            feedforward = Feedforward.Type.ARM;

        if (currentSlot.gravityType() == GravityTypeValue.Elevator_Static)
            feedforward = Feedforward.Type.ELEVATOR;

        feedforward.setFeedforwardConstants(
                currentSlot.kS(),
                currentSlot.kV(),
                currentSlot.kA(),
                currentSlot.kG()
        );
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
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, 32767);
    }

    private void handleSmoothMotion(MotorProperties.ControlMode controlMode, double feedforward) {
        double feedforwardOutput, feedbackOutput, acceleration;

        feedbackOutput = getModeBasedFeedback(controlMode, goalState);
        feedforwardOutput = this.feedforward.calculate(goalState.position, goalState.velocity, 0);

        if (positionMotionProfile != null && controlMode == MotorProperties.ControlMode.POSITION) {
            final TrapezoidProfile.State currentSetpoint = positionMotionProfile.calculate(0.02, previousSetpoint, goalState);

            acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;

            feedforwardOutput = this.feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity, acceleration);
            feedbackOutput = feedback.calculate(getEffectivePosition(), currentSetpoint.position);

            previousSetpoint = currentSetpoint;
        }

        if (velocityMotionProfile != null && controlMode == MotorProperties.ControlMode.VELOCITY) {
            final TrapezoidProfile.State currentSetpoint = velocityMotionProfile.calculate(0.02, previousSetpoint, goalState);
            //Position -> velocity. Velocity -> acc.
            feedforwardOutput = this.feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity);
            feedbackOutput = feedback.calculate(getEffectiveVelocity(), currentSetpoint.position);

            previousSetpoint = currentSetpoint;
        }

        if (feedforward != USE_BUILTIN_FEEDFORWARD_NUMBER) feedforwardOutput = feedforward;

        controller.setReference(feedforwardOutput + feedbackOutput, CANSparkBase.ControlType.kVoltage);
    }

    private void setGoal(MotorProperties.ControlMode controlMode, double output) {
        final TrapezoidProfile.State stateFromOutput = new TrapezoidProfile.State(output, 0);

        if (goalState == null || !goalState.equals(stateFromOutput)) {
            feedback.reset();

            if (controlMode == MotorProperties.ControlMode.POSITION)
                previousSetpoint = new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity());
            else if (controlMode == MotorProperties.ControlMode.VELOCITY)
                previousSetpoint = new TrapezoidProfile.State(getEffectiveVelocity(), getEffectiveAcceleration());

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

    private double getEffectivePosition() {
        return externalPositionSupplier == null ? getSystemPositionPrivate() : externalPositionSupplier.getAsDouble();
    }

    private double getEffectiveVelocity() {
        return externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();
    }

    private double getEffectiveAcceleration() {
        final double acceleration = externalVelocitySupplier == null ? getSystemVelocityPrivate() - previousVelocity : externalVelocitySupplier.getAsDouble() - previousVelocity;

        previousVelocity = externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();

        return acceleration;
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces">REV DOCS</a>
     */
    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        final int ms = 1000 / (useFasterThread ? 200 : 50);

        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case VELOCITY, CURRENT, TEMPERATURE -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, ms);
            case POSITION -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, ms);
        }

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + MOTOR_INPUTS_LENGTH / 2] = true;

        switch (signal) {
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
    protected void refreshInputs(MotorInputs inputs) {
        if (spark == null) return;

        inputs.setSignalsToLog(signalsToLog);

        if (signalsToLog[0]) inputs.voltage = getVoltagePrivate();
        if (signalsToLog[1]) inputs.current = spark.getOutputCurrent();
        if (signalsToLog[2]) inputs.temperature = spark.getMotorTemperature();
        if (signalsToLog[3]) inputs.target = closedLoopTarget;
        if (signalsToLog[4]) inputs.systemPosition = getEffectivePosition();
        if (signalsToLog[5]) inputs.systemVelocity = getEffectiveVelocity();

        MotorUtilities.handleThreadedInputs(inputs, signalQueueList);
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
