package frc.lib.generic.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;

public class TestSpark extends CANSparkBase implements Motor {
    private final MotorProperties.SparkType model;
    private final RelativeEncoder encoder;

    private final SparkPIDController builtinController;

    private MotorConfiguration currentConfiguration;

    private double closedLoopTarget;

    private ProfiledPIDController controller;
    private Feedforward feedforward;

    private int slotToUse = 0;

    public TestSpark(int deviceId, MotorProperties.SparkType sparkType) {
        super(deviceId, MotorType.kBrushless, sparkType == MotorProperties.SparkType.MAX ? SparkModel.SparkMax : SparkModel.SparkFlex);
        model = sparkType;

        optimizeBusUsage();

        encoder = this.getEncoder();
        builtinController = super.getPIDController();

        controller = null;
    }

    private void setGoal(double output) {
        if(Objects.equals(controller.getGoal(), new TrapezoidProfile.State(output, 0))) {
            return;
        }

        controller.setGoal(new TrapezoidProfile.State(output, 0));
//        controller.reset(getSystemPosition());
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        if (controller == null) return;

        setGoal(output);

        double ffOutput = feedforward.calculate(
                controller.getSetpoint().position,
                controller.getSetpoint().velocity,
                0
        );

        Logger.recordOutput("ArmFF FF", ffOutput);
        Logger.recordOutput("ArmFF SETPOINT POSITION", controller.getSetpoint().position);
        Logger.recordOutput("ArmFF SETPOINT VELOCITY", controller.getSetpoint().velocity);

        setOutput(controlMode, output, ffOutput);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;

        switch (mode) {
            case PERCENTAGE_OUTPUT -> builtinController.setReference(output, ControlType.kDutyCycle);

            case VELOCITY -> {
//                controller.setReference(output * 60, ControlType.kVelocity, slotToUse, feedforward);
            }
            case POSITION -> {
                double controllerOutput = controller.calculate(getSystemPosition());

                Logger.recordOutput("ArmFF CONTROLLER", controllerOutput);
                Logger.recordOutput("ArmFF system position", getSystemPosition());


                setOutput(MotorProperties.ControlMode.VOLTAGE, controllerOutput + feedforward);
//                controller.setReference(output, ControlType.kPosition, slotToUse);
            }

            case VOLTAGE -> builtinController.setReference(output, ControlType.kVoltage, slotToUse);
            case CURRENT -> builtinController.setReference(output, ControlType.kCurrent, slotToUse);
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
        return encoder.getPosition() / currentConfiguration.gearRatio;
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

        controller = new ProfiledPIDController(
                currentConfiguration.slot0.kP(), currentConfiguration.slot0.kI(), currentConfiguration.slot0.kD(),

                new TrapezoidProfile.Constraints(
                        currentConfiguration.profiledMaxVelocity, currentConfiguration.profiledTargetAcceleration

                ));

        configureProfile(configuration);

        configurePID(configuration);
        configureFeedForward(configuration);

        return super.burnFlash() == REVLibError.kOk;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profiledMaxVelocity == 0 || configuration.profiledTargetAcceleration == 0) return;

        builtinController.setSmartMotionMaxVelocity(configuration.profiledMaxVelocity / 60, slotToUse);
        builtinController.setSmartMotionMaxAccel(configuration.profiledTargetAcceleration / 60, slotToUse);

//        controller.setSmartMotionAllowedClosedLoopError(configuration.closedLoopError, slotToUse);//todo: Might be needed. do test.
        builtinController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, slotToUse); //todo: add a way to edit this. only if needed tho. meh
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
    }

    private void configurePID(MotorConfiguration configuration) {
        builtinController.setPositionPIDWrappingEnabled(configuration.closedLoopContinuousWrap);

        builtinController.setP(configuration.slot0.kP(), 0);
        builtinController.setI(configuration.slot0.kI(), 0);
        builtinController.setD(configuration.slot0.kD(), 0);

        builtinController.setP(configuration.slot1.kP(), 1);
        builtinController.setI(configuration.slot1.kI(), 1);
        builtinController.setD(configuration.slot1.kD(), 1);

        builtinController.setP(configuration.slot2.kP(), 2);
        builtinController.setI(configuration.slot2.kI(), 2);
        builtinController.setD(configuration.slot2.kD(), 2);

        slotToUse = configuration.slotToUse;
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
}
