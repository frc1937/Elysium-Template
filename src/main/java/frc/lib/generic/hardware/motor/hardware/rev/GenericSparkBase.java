package frc.lib.generic.hardware.motor.hardware.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
import frc.lib.scurve.InputParameter;
import frc.lib.scurve.OutputParameter;
import frc.lib.scurve.SCurveGenerator;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;

public abstract class GenericSparkBase extends Motor {
    private MotorUtilities.MotionType motionType;

    private final CANSparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkPIDController sparkController;
    private final int deviceId;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private DoubleSupplier externalPositionSupplier, externalVelocitySupplier;
    private Feedforward feedforward;

    private double previousVelocity = 0;

    private SCurveGenerator scurveGenerator;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State goalState;

    private boolean hasStoppedOccurred = false;

    private MotorConfiguration currentConfiguration;

    private int slotToUse = 0;
    private double conversionFactor = 1;

    protected GenericSparkBase(String name, int deviceId) {
        super(name);

        this.deviceId = deviceId;

        spark = getSpark();
        encoder = getEncoder();
        sparkController = getSparkController();

        optimizeBusUsage(spark);
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
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        setNewGoal(output);

        switch (mode) {
            case POSITION, VELOCITY -> handleSmoothMotion(motionType, goalState, motionProfile, this.feedforward, slotToUse);
            case VOLTAGE -> sparkController.setReference(output, CANSparkBase.ControlType.kVoltage, slotToUse, 0);
            case CURRENT -> sparkController.setReference(output, CANSparkBase.ControlType.kCurrent, slotToUse, 0);
        }
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        slotToUse = configuration.slotToUse;

        spark.restoreFactoryDefaults();

        setIdleMode(configuration.idleMode);
        spark.setInverted(configuration.inverted);

        spark.enableVoltageCompensation(12);

        conversionFactor = (1.0 / configuration.gearRatio);

        if (configuration.statorCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureFeedforward(getCurrentSlot());

        configureProfile(configuration);
        configurePID(configuration);
        configureExtras(configuration);

        int i = 0;

        while (i <= 5 && spark.burnFlash() != REVLibError.kOk) {
            i++;
        }

        return spark.burnFlash() == REVLibError.kOk;
    }

    protected void configureFeedforward(MotorProperties.Slot slot) {
        final Feedforward.Type type = switch (slot.gravityType()) {
            case ARM -> Feedforward.Type.ARM;
            case ELEVATOR -> Feedforward.Type.ELEVATOR;
            default -> Feedforward.Type.SIMPLE;
        };

        this.feedforward = new Feedforward(type,
                new Feedforward.FeedForwardConstants(slot.kS(), slot.kV(), slot.kA(), slot.kG()));
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
        return deviceId;
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        spark.setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
    }

    private void setNewGoal(double goal) {
        if (hasNoNewGoal(new TrapezoidProfile.State(goal, 0))) return;

        hasStoppedOccurred = false;
        setNewGoalExtras();

        if (motionType == MotorUtilities.MotionType.POSITION_TRAPEZOIDAL) {
            setPreviousSetpoint(new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity()));
        } else if (motionType == MotorUtilities.MotionType.VELOCITY_TRAPEZOIDAL) {
            setPreviousSetpoint(new TrapezoidProfile.State(getEffectiveVelocity(), getEffectiveAcceleration()));
        } else if (motionType == MotorUtilities.MotionType.POSITION_S_CURVE) {
            setSCurveInputs(new InputParameter(
                    getEffectivePosition(),
                    getEffectiveVelocity(),
                    getEffectiveAcceleration(),
                    goal
            ));

            setSCurveOutputs(new OutputParameter());
        }

        goalState = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces">REV DOCS</a>
     */
    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        final int ms = 1000 / (useFasterThread ? 200 : 50);

        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case VELOCITY, CURRENT, TEMPERATURE, ACCELERATION ->
                    spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, ms);
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
                    signalQueueList.put("target", OdometryThread.getInstance().registerSignal(() -> goalState.position));
            case ACCELERATION ->
                    signalQueueList.put("acceleration", OdometryThread.getInstance().registerSignal(this::getEffectiveAcceleration));
        }
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (spark == null) return;

        refreshExtras();

        inputs.setSignalsToLog(signalsToLog);

        inputs.voltage = getVoltagePrivate();
        inputs.current = spark.getOutputCurrent();
        inputs.temperature = spark.getMotorTemperature();
        if (goalState != null) inputs.target = goalState.position;
        inputs.systemPosition = getEffectivePosition();
        inputs.systemVelocity = getEffectiveVelocity();
        inputs.systemAcceleration = getEffectiveAcceleration();

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

    double getEffectivePosition() {
        return externalPositionSupplier == null ? getSystemPositionPrivate() : externalPositionSupplier.getAsDouble();
    }

    double getEffectiveVelocity() {
        return externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();
    }

    private double getEffectiveAcceleration() {
        final double acceleration = externalVelocitySupplier == null ? getSystemVelocityPrivate() - previousVelocity : externalVelocitySupplier.getAsDouble() - previousVelocity;

        previousVelocity = externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();

        return acceleration;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profileMaxVelocity != 0 && configuration.profileMaxAcceleration != 0) {
            if (configuration.profileMaxJerk != 0) {
                scurveGenerator = new SCurveGenerator(0.02,
                        configuration.profileMaxVelocity,
                        configuration.profileMaxAcceleration,
                        configuration.profileMaxJerk);

                motionType = MotorUtilities.MotionType.POSITION_S_CURVE;
            } else {
                motionProfile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                configuration.profileMaxVelocity,
                                configuration.profileMaxAcceleration
                        )
                );

                motionType = MotorUtilities.MotionType.POSITION_TRAPEZOIDAL;
            }
        } else if (configuration.profileMaxAcceleration != 0 && configuration.profileMaxJerk != 0) {
            motionProfile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            configuration.profileMaxAcceleration,
                            configuration.profileMaxJerk
                        )
                );

            motionType = MotorUtilities.MotionType.VELOCITY_TRAPEZOIDAL;
        } else if (feedforward.getConstants().kG != 0 && feedforward.getConstants().kV == 0 && feedforward.getConstants().kA == 0 && feedforward.getConstants().kS == 0) {
            motionType = MotorUtilities.MotionType.POSITION_PID_WITH_KG;
        } else if (feedforward.getConstants().kS == 0 && feedforward.getConstants().kG == 0 && feedforward.getConstants().kV == 0 && feedforward.getConstants().kA == 0) {
            motionType = MotorUtilities.MotionType.POSITION_PID;
        } else {
            motionType = MotorUtilities.MotionType.VELOCITY_PID_FF;
        }
    }

    private boolean hasNoNewGoal(TrapezoidProfile.State newGoal) {
        return goalState != null
                && goalState.equals(newGoal)
                && !hasStoppedOccurred
                && (Logger.getRealTimestamp() - getLastProfileCalculationTimestamp() <= 100000); //(0.1 sec has passed)
    }

    protected SCurveGenerator getSCurveGenerator() {
        return scurveGenerator;
    }

    protected abstract void setSCurveInputs(InputParameter scurveInputs);

    protected abstract void setSCurveOutputs(OutputParameter outputParameter);

    protected abstract CANSparkBase getSpark();

    protected abstract RelativeEncoder getEncoder();

    protected abstract SparkPIDController getSparkController();

    protected abstract void refreshExtras();

    protected abstract void setNewGoalExtras();

    protected abstract void configureExtras(MotorConfiguration configuration);

    protected abstract void handleSmoothMotion(MotorUtilities.MotionType motionType, TrapezoidProfile.State goalState, TrapezoidProfile motionProfile, final Feedforward feedforward, int slotToUse);

    protected abstract void configurePID(MotorConfiguration configuration);

    protected abstract double getLastProfileCalculationTimestamp();

    protected abstract void setPreviousSetpoint(TrapezoidProfile.State previousSetpoint);

    private void optimizeBusUsage(CANSparkBase spark) {
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
