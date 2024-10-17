package frc.lib.generic.hardware.motor.hardware.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
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

public class GenericSparkFlex extends Motor {
    private final CANSparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkPIDController sparkController;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private double closedLoopTarget;

    private MotorConfiguration currentConfiguration;
    private Feedforward.Type feedforward;

    private int slotToUse = 0;
    private double conversionFactor = 1;

    private DoubleSupplier externalPositionSupplier, externalVelocitySupplier;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State previousSetpoint, goalState;

    private double previousVelocity = 0;
    private boolean hasStoppedOccurred = false;
    private double lastProfileCalculationTimestamp;

    private final Timer timer = new Timer();

    public GenericSparkFlex(String name, int deviceId) {
        super(name);

        spark = new CANSparkFlex(deviceId, CANSparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();
        sparkController = spark.getPIDController();

        SparkCommon.optimizeBusUsage(spark);

        timer.start();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setNewGoal(mode, output);

        switch (mode) {
            case PERCENTAGE_OUTPUT -> sparkController.setReference(output, CANSparkBase.ControlType.kDutyCycle);
            case POSITION, VELOCITY -> handleSmoothMotion(mode);
            case VOLTAGE -> sparkController.setReference(output, CANSparkBase.ControlType.kVoltage, slotToUse, 0);
            case CURRENT -> sparkController.setReference(output, CANSparkBase.ControlType.kCurrent, slotToUse, 0);
        }
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
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        spark.setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        spark.restoreFactoryDefaults();

        setIdleMode(configuration.idleMode);
        spark.setInverted(configuration.inverted);

        spark.enableVoltageCompensation(12);

        encoder.setPositionConversionFactor(1 / configuration.gearRatio);
        encoder.setVelocityConversionFactor(1/ (configuration.gearRatio));

        conversionFactor = (1.0 / configuration.gearRatio);

        if (configuration.statorCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureProfile(configuration);
        configurePID(configuration);

        feedforward = SparkCommon.configureFeedforward(getCurrentSlot());

        encoder.setPosition(getEffectivePosition());

        return spark.burnFlash() == REVLibError.kOk;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profiledMaxVelocity != 0 && configuration.profiledTargetAcceleration != 0) {
            motionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            configuration.profiledMaxVelocity,
                            configuration.profiledTargetAcceleration
                    )
            );
        }

        if (configuration.profiledTargetAcceleration != 0 && configuration.profiledJerk != 0) {
            motionProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            configuration.profiledTargetAcceleration,
                            configuration.profiledJerk
                    )
            );
        }
    }

    private void configurePID(MotorConfiguration configuration) {
        slotToUse = configuration.slotToUse;

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

    private void handleSmoothMotion(MotorProperties.ControlMode controlMode) {
        if (goalState == null) return;

        double feedforwardOutput, acceleration;
        final CANSparkBase.ControlType controlType = controlMode == MotorProperties.ControlMode.POSITION ? CANSparkBase.ControlType.kPosition : CANSparkBase.ControlType.kVelocity;

        if (motionProfile != null) {
            final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

            if (controlMode == MotorProperties.ControlMode.POSITION) {
                acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;
                feedforwardOutput = feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity, acceleration);

                sparkController.setReference(currentSetpoint.position,
                        CANSparkBase.ControlType.kPosition,
                        slotToUse, feedforwardOutput,
                        SparkPIDController.ArbFFUnits.kVoltage);
            }

            if (controlMode == MotorProperties.ControlMode.VELOCITY) {
                feedforwardOutput = feedforward.calculate(0, currentSetpoint.position, currentSetpoint.velocity);

                sparkController.setReference(currentSetpoint.position * 60,
                        CANSparkBase.ControlType.kVelocity,
                        slotToUse, feedforwardOutput,
                        SparkPIDController.ArbFFUnits.kVoltage);
            }


            previousSetpoint = currentSetpoint;
            lastProfileCalculationTimestamp = Logger.getRealTimestamp();
        } else {
            feedforwardOutput = feedforward.calculate(goalState.position, goalState.velocity, 0);

            final double goal = controlType == CANSparkBase.ControlType.kPosition ? goalState.position : 60 * goalState.position;

            sparkController.setReference(goal,
                    controlType, slotToUse, feedforwardOutput,
                    SparkPIDController.ArbFFUnits.kVoltage);
        }
    }

    private void setNewGoal(MotorProperties.ControlMode controlMode, double goal) {
        if (SparkCommon.hasNoNewGoal(new TrapezoidProfile.State(goal, 0), goalState, hasStoppedOccurred, lastProfileCalculationTimestamp)) return;

        hasStoppedOccurred = false;

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

        if (timer.advanceIfElapsed(4)) {
            encoder.setPosition(getEffectivePosition());
        }

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
}
