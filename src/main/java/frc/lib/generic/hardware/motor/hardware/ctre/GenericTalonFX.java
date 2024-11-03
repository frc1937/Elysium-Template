package frc.lib.generic.hardware.motor.hardware.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.motor.hardware.MotorUtilities;

import java.util.*;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorProperties.GravityType.ARM;

public class GenericTalonFX extends Motor {
    private final TalonFX talonFX;

    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private final boolean[] signalsToLog = new boolean[MotorInputs.MOTOR_INPUTS_LENGTH];
    private final StatusSignal<Double> positionSignal, velocitySignal, accelerationSignal, voltageSignal, currentSignal, temperatureSignal;
    private final List<StatusSignal<Double>> signalsToUpdateList = new ArrayList<>();

    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private MotorConfiguration currentConfiguration;

    private boolean shouldUseProfile = false;
    private int slotToUse = 0;

    private double target;

    public GenericTalonFX(String name, int deviceId) {
        super(name);

        talonFX = new TalonFX(deviceId);

        talonConfigurator = talonFX.getConfigurator();

        positionSignal = talonFX.getPosition().clone();
        velocitySignal = talonFX.getVelocity().clone();
        accelerationSignal = talonFX.getAcceleration().clone();
        voltageSignal = talonFX.getMotorVoltage().clone();
        currentSignal = talonFX.getStatorCurrent().clone();
        temperatureSignal = talonFX.getDeviceTemp().clone();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        target = output;

        switch (mode) {
            case PERCENTAGE_OUTPUT -> talonFX.setControl(dutyCycleRequest.withOutput(output));
            case VOLTAGE -> talonFX.setControl(voltageRequest.withOutput(output));

            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(slotToUse));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(slotToUse));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(slotToUse));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(slotToUse));
                }
            }

            case CURRENT -> new UnsupportedOperationException("CTRE LOVES money and wants $150!!! dollars for this.. wtf.").printStackTrace();
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        if (mode != MotorProperties.ControlMode.POSITION && mode != MotorProperties.ControlMode.VELOCITY)
            setOutput(mode, output);

        target = output;

        switch (mode) {
            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(slotToUse).withFeedForward(feedforward));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(slotToUse).withFeedForward(feedforward));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(slotToUse).withFeedForward(feedforward));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(slotToUse).withFeedForward(feedforward));
                }
            }
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        currentConfiguration.idleMode = idleMode;
        configure(currentConfiguration);
    }

    @Override
    public void stopMotor() {
        talonFX.stopMotor();
    }

    @Override
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) {
        new UnsupportedOperationException("Setting external position source is not supported in TalonFX.\n Periodically call #setMotorEncoderPosition() instead.").printStackTrace();
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) {
        new UnsupportedOperationException("Setting external velocity source is not supported in TalonFX").printStackTrace();
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        talonConfigurator.setPosition(position); //TODO: Test on real robot to check if works.
    }

    @Override
    public int getDeviceID() {
        return talonFX.getDeviceID();
    }

    @Override
    public void setFollowerOf(String name, int masterPort) {
        talonFX.setControl(new StrictFollower(masterPort)); //check if this should be called 10 times or once is enough
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        this.currentConfiguration = configuration;

        talonConfig.MotorOutput.Inverted = configuration.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //Who the FUCK added this feature. CTRE should fucking fire him bruh
        talonConfig.Audio.BeepOnBoot = false;
        talonConfig.Audio.BeepOnConfig = false;

        talonConfig.Voltage.PeakForwardVoltage = 12;
        talonConfig.Voltage.PeakReverseVoltage = -12;

        talonConfig.Feedback.SensorToMechanismRatio = configuration.gearRatio;

        configureMotionMagic();

        setConfig0();
        setConfig1();
        setConfig2();

        applyCurrentLimits();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

        slotToUse = configuration.slotToUse;

        talonFX.optimizeBusUtilization();

        return applyConfig();
    }

    private void configureMotionMagic() {
        if (currentConfiguration.profiledMaxVelocity == 0 || currentConfiguration.profiledTargetAcceleration == 0)
            return;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = currentConfiguration.profiledMaxVelocity;
        talonConfig.MotionMagic.MotionMagicAcceleration = currentConfiguration.profiledTargetAcceleration;
        talonConfig.MotionMagic.MotionMagicJerk = currentConfiguration.profiledJerk;

        shouldUseProfile = true;
    }

    private void setConfig0() {
        talonConfig.Slot0.kP = currentConfiguration.slot0.kP();
        talonConfig.Slot0.kI = currentConfiguration.slot0.kI();
        talonConfig.Slot0.kD = currentConfiguration.slot0.kD();

        talonConfig.Slot0.kA = currentConfiguration.slot0.kA();
        talonConfig.Slot0.kS = currentConfiguration.slot0.kS();
        talonConfig.Slot0.kV = currentConfiguration.slot0.kV();
        talonConfig.Slot0.kG = currentConfiguration.slot0.kG();

        if (currentConfiguration.slot0.gravityType() != null)
            talonConfig.Slot0.GravityType = currentConfiguration.slot0.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;

        if (currentConfiguration.slot0.kS() != 0)
            talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    }

    private void setConfig1() {
        talonConfig.Slot1.kP = currentConfiguration.slot1.kP();
        talonConfig.Slot1.kI = currentConfiguration.slot1.kI();
        talonConfig.Slot1.kD = currentConfiguration.slot1.kD();
        talonConfig.Slot1.kA = currentConfiguration.slot1.kA();
        talonConfig.Slot1.kS = currentConfiguration.slot1.kS();
        talonConfig.Slot1.kV = currentConfiguration.slot1.kV();
        talonConfig.Slot1.kG = currentConfiguration.slot1.kG();

        if (currentConfiguration.slot1.gravityType() != null)
            talonConfig.Slot1.GravityType = currentConfiguration.slot1.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;


        if (currentConfiguration.slot0.kS() != 0)
            talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    }

    private void setConfig2() {
        talonConfig.Slot2.kP = currentConfiguration.slot2.kP();
        talonConfig.Slot2.kI = currentConfiguration.slot2.kI();
        talonConfig.Slot2.kD = currentConfiguration.slot2.kD();
        talonConfig.Slot2.kA = currentConfiguration.slot2.kA();
        talonConfig.Slot2.kS = currentConfiguration.slot2.kS();
        talonConfig.Slot2.kV = currentConfiguration.slot2.kV();
        talonConfig.Slot2.kG = currentConfiguration.slot2.kG();

        if (currentConfiguration.slot2.gravityType() != null)
            talonConfig.Slot2.GravityType = currentConfiguration.slot2.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;

        if (currentConfiguration.slot0.kS() != 0)
            talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    }

    private void applyCurrentLimits() {
        talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = currentConfiguration.dutyCycleOpenLoopRampPeriod;
        talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = currentConfiguration.dutyCycleClosedLoopRampPeriod;

        if (currentConfiguration.statorCurrentLimit != -1) {
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfig.CurrentLimits.StatorCurrentLimit = currentConfiguration.statorCurrentLimit;
        }

        if (currentConfiguration.supplyCurrentLimit != -1) {
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            talonConfig.CurrentLimits.SupplyCurrentLimit = currentConfiguration.supplyCurrentLimit;
        }
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = talonConfigurator.apply(talonConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }

    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        final int updateFrequency = useFasterThread ? 200 : 50;

        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case VELOCITY -> setupSignal(velocitySignal, updateFrequency);
            case POSITION -> setupSignal(positionSignal, updateFrequency);
            case ACCELERATION -> setupSignal(accelerationSignal, updateFrequency);
            case VOLTAGE -> setupSignal(voltageSignal, updateFrequency);
            case CURRENT -> setupSignal(currentSignal, updateFrequency);
            case TEMPERATURE -> setupSignal(temperatureSignal, updateFrequency);
        }

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + MotorInputs.MOTOR_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case VELOCITY -> signalQueueList.put("velocity", OdometryThread.getInstance().registerSignal(this::getSystemVelocityPrivate));
            case POSITION -> signalQueueList.put("position", OdometryThread.getInstance().registerSignal(this::getSystemPositionPrivate));
            case ACCELERATION -> signalQueueList.put("acceleration", OdometryThread.getInstance().registerSignal(this::getSystemAccelerationPrivate));
            case VOLTAGE -> signalQueueList.put("voltage", OdometryThread.getInstance().registerSignal(this::getVoltagePrivate));
            case CURRENT -> signalQueueList.put("current", OdometryThread.getInstance().registerSignal(this::getCurrentPrivate));
            case TEMPERATURE -> signalQueueList.put("temperature", OdometryThread.getInstance().registerSignal(this::getTemperaturePrivate));
        }
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (talonFX == null) return;

        inputs.setSignalsToLog(signalsToLog);

        BaseStatusSignal.refreshAll(signalsToUpdateList.toArray(new BaseStatusSignal[0]));

        inputs.voltage = getVoltagePrivate();
        inputs.current = getCurrentPrivate();
        inputs.temperature = getTemperaturePrivate();
        inputs.target = target;
        inputs.systemPosition = getSystemPositionPrivate();
        inputs.systemVelocity = getSystemVelocityPrivate();
        inputs.systemAcceleration = getSystemAccelerationPrivate();

        MotorUtilities.handleThreadedInputs(inputs, signalQueueList);
    }

    private double getSystemPositionPrivate() {
        return positionSignal.getValue();
    }

    private double getSystemVelocityPrivate() {
        return velocitySignal.getValue();
    }

    private double getSystemAccelerationPrivate() { return accelerationSignal.getValue(); }

    private double getVoltagePrivate() {
        return voltageSignal.getValue();
    }

    private double getTemperaturePrivate() {
        return temperatureSignal.getValue();
    }

    private double getCurrentPrivate() {
        return currentSignal.getValue();
    }

    private void setupSignal(final StatusSignal<Double> correspondingSignal, final int updateFrequency) {
        correspondingSignal.setUpdateFrequency(updateFrequency);
        signalsToUpdateList.add(correspondingSignal);
    }
}
