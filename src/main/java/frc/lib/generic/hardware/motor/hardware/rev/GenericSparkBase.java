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

//todo: possible problem when doing PID only velocity control.
public abstract class GenericSparkBase extends Motor {
    private SparkCommon.MotionType motionType;

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

    private double kS, kV, kA, kG;

    protected GenericSparkBase(String name, int deviceId) {
        super(name);

        this.deviceId = deviceId;

        spark = getSpark();
        encoder = getEncoder();
        sparkController = getSparkController();

        SparkCommon.optimizeBusUsage(spark);
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
            case PERCENTAGE_OUTPUT -> sparkController.setReference(output, CANSparkBase.ControlType.kDutyCycle);
            case POSITION, VELOCITY -> {
                if (getDeviceID() == 28) {
                    this.feedforward = new Feedforward(Feedforward.Type.SIMPLE, new Feedforward.FeedForwardConstants(kS, kV, kA, kG));
//                    this.feedforward.setFeedforwardConstants(kS, kV, kA, kG);
                }
                //todo wtf.. change how you do FF so this doesn;t happen.
                handleSmoothMotion(motionType, goalState, motionProfile, this.feedforward, slotToUse);
            }
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
        final Feedforward.FeedForwardConstants constants = new Feedforward.FeedForwardConstants(slot.kS(), slot.kV(), slot.kA(), slot.kG());

        this.feedforward = new Feedforward(Feedforward.Type.SIMPLE, constants);

        if (slot.gravityType() == MotorProperties.GravityType.ARM) {
            this.feedforward = new Feedforward(Feedforward.Type.ARM, constants);
        }

        if (slot.gravityType() == MotorProperties.GravityType.ELEVATOR) {
            this.feedforward = new Feedforward(Feedforward.Type.ELEVATOR, constants);
        }

        this.kS = slot.kS();
        this.kV = slot.kV();
        this.kA = slot.kA();
        this.kG = slot.kG();
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

        if (motionType == SparkCommon.MotionType.POSITION_TRAPEZOIDAL) {
            setPreviousSetpoint(new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity()));
        } else if (motionType == SparkCommon.MotionType.VELOCITY_TRAPEZOIDAL) {
            setPreviousSetpoint(new TrapezoidProfile.State(getEffectiveVelocity(), getEffectiveAcceleration()));
        } else if (motionType == SparkCommon.MotionType.POSITION_S_CURVE) {
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

        if (signalsToLog[0]) inputs.voltage = getVoltagePrivate();
        if (signalsToLog[1]) inputs.current = spark.getOutputCurrent();
        if (signalsToLog[2]) inputs.temperature = spark.getMotorTemperature();
        if (signalsToLog[3] && goalState != null) inputs.target = goalState.position;
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
        if (configuration.profiledMaxVelocity != 0 && configuration.profiledTargetAcceleration != 0) {
            if (configuration.profiledJerk != 0) {
                scurveGenerator = new SCurveGenerator(0.02,
                        configuration.profiledMaxVelocity,
                        configuration.profiledTargetAcceleration,
                        configuration.profiledJerk);

                motionType = SparkCommon.MotionType.POSITION_S_CURVE;
            } else {
                motionProfile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                configuration.profiledMaxVelocity,
                                configuration.profiledTargetAcceleration
                        )
                );

                motionType = SparkCommon.MotionType.POSITION_TRAPEZOIDAL;
            }
        } else if (configuration.profiledTargetAcceleration != 0 && configuration.profiledJerk != 0) {
            motionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            configuration.profiledTargetAcceleration,
                            configuration.profiledJerk
                    )
            );

            motionType = SparkCommon.MotionType.VELOCITY_TRAPEZOIDAL;
        } else if (feedforward.getConstants().kS == 0 && feedforward.getConstants().kG == 0 && feedforward.getConstants().kV == 0 && feedforward.getConstants().kA == 0) {
            motionType = SparkCommon.MotionType.POSITION_PID;
        } else {
            motionType = SparkCommon.MotionType.VELOCITY_PID_FF;
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

    protected abstract void handleSmoothMotion(SparkCommon.MotionType motionType, TrapezoidProfile.State goalState, TrapezoidProfile motionProfile, final Feedforward feedforward, int slotToUse);

    protected abstract void configurePID(MotorConfiguration configuration);

    protected abstract double getLastProfileCalculationTimestamp();

    protected abstract void setPreviousSetpoint(TrapezoidProfile.State previousSetpoint);
}
