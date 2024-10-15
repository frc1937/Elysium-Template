package frc.lib.generic.hardware.motor.hardware.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.hardware.motor.hardware.MotorUtilities;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;

public class GenericSparkMax extends Motor {
    private final CANSparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkPIDController sparkController;

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
    private boolean hasStoppedOccurred = false;
    private double lastProfileCalculationTimestamp;

    public GenericSparkMax(String name, int deviceId) {
        super(name);

        spark = new CANSparkMax(deviceId, CANSparkFlex.MotorType.kBrushless);
        encoder = spark.getEncoder();
        sparkController = spark.getPIDController();

        optimizeBusUsage();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setGoal(mode, output);

        switch (mode) {
            case PERCENTAGE_OUTPUT -> sparkController.setReference(output, CANSparkBase.ControlType.kDutyCycle);
            case POSITION, VELOCITY -> handleSmoothMotion(mode);
            case VOLTAGE -> sparkController.setReference(output, CANSparkBase.ControlType.kVoltage, slotToUse, 0);
            case CURRENT -> sparkController.setReference(output, CANSparkBase.ControlType.kCurrent, slotToUse, 0);
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        spark.setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public void stopMotor() {
        hasStoppedOccurred = true;
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
        if (configuration.profiledMaxVelocity != 0 && configuration.profiledTargetAcceleration != 0) {
            positionMotionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            configuration.profiledMaxVelocity,
                            configuration.profiledTargetAcceleration
                    ));
        }

        if (configuration.profiledTargetAcceleration != 0 && configuration.profiledJerk != 0) {
            velocityMotionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            configuration.profiledTargetAcceleration,
                            configuration.profiledJerk
                    ));
        }
    }

    private void configureFeedForward() {
        final MotorProperties.Slot currentSlot = getCurrentSlot();

        feedforward = Feedforward.Type.SIMPLE;

        if (currentSlot.gravityType() == MotorProperties.GravityType.ARM) {
            feedforward = Feedforward.Type.ARM;
        }

        if (currentSlot.gravityType() == MotorProperties.GravityType.ELEVATOR) {
            feedforward = Feedforward.Type.ELEVATOR;
        }

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

    private void handleSmoothMotion(MotorProperties.ControlMode controlMode) {
        double feedbackOutput, feedforwardOutput, acceleration;

        if (positionMotionProfile != null && controlMode == MotorProperties.ControlMode.POSITION) {
            if (goalState == null) return;

            final TrapezoidProfile.State currentSetpoint = positionMotionProfile.calculate(0.02, previousSetpoint, goalState);

            acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;

            feedforwardOutput = this.feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity, acceleration);
            feedbackOutput = feedback.calculate(getEffectivePosition(), currentSetpoint.position);

            previousSetpoint = currentSetpoint;
            lastProfileCalculationTimestamp = Logger.getTimestamp();
        } else if (velocityMotionProfile != null && controlMode == MotorProperties.ControlMode.VELOCITY) {
            if (goalState == null) return;

            final TrapezoidProfile.State currentSetpoint = velocityMotionProfile.calculate(0.02, previousSetpoint, goalState);

            feedforwardOutput = this.feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity);
            feedbackOutput = this.feedback.calculate(getEffectiveVelocity(), currentSetpoint.position);

            previousSetpoint = currentSetpoint;
            lastProfileCalculationTimestamp = Logger.getRealTimestamp();
        } else {
            if (goalState == null) return;

            feedbackOutput = getModeBasedFeedback(controlMode, goalState);
            feedforwardOutput = this.feedforward.calculate(goalState.position, goalState.velocity, 0);
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

    private void setGoal(MotorProperties.ControlMode controlMode, double goal) {
        System.out.println("SHOULD RESET PROFILE " + getName() + "I: " + shouldResetProfile(new TrapezoidProfile.State(goal, 0)));

        if (!shouldResetProfile(new TrapezoidProfile.State(goal, 0))) return;

        hasStoppedOccurred = false;
        feedback.reset();

        if (controlMode == MotorProperties.ControlMode.POSITION)
            previousSetpoint = new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity());
        else if (controlMode == MotorProperties.ControlMode.VELOCITY)
            previousSetpoint = new TrapezoidProfile.State(getEffectiveVelocity(), getEffectiveAcceleration());

        goalState = new TrapezoidProfile.State(goal, 0);
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
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
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
        if (signalsToLog[6]) inputs.systemAcceleration = getEffectiveAcceleration();

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

    private boolean shouldResetProfile(TrapezoidProfile.State newGoal) {
        return goalState == null
                || !goalState.equals(newGoal)
                || hasStoppedOccurred
                || (Logger.getRealTimestamp() - lastProfileCalculationTimestamp) > 100000; //(0.1 sec has passed)
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
}
