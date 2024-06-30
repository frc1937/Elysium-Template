//package frc.robot.subsystems;
//
//import edu.wpi.first.units.Measure;
//import edu.wpi.first.units.Voltage;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//import frc.lib.generic.motor.MotorProperties;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.concurrent.CompletableFuture;
//
//public abstract class GenericSubsystem extends SubsystemBase {
//    //This variable is shared across all instances of the abstract class.
//    private static final List<GenericSubsystem> REGISTERED_SUBSYSTEMS = new ArrayList<>();
//    private static final Trigger IS_ROBOT_DISABLED = new Trigger(DriverStation::isDisabled);
//
//    static {
//        IS_ROBOT_DISABLED.onTrue(Commands.runOnce(() -> {
//                    REGISTERED_SUBSYSTEMS.forEach(GenericSubsystem::stop);
//                    REGISTERED_SUBSYSTEMS.forEach((subsystem) -> subsystem.setIdleMode(defaultDisableState));
//                })
//                .ignoringDisable(true));
//
//        IS_ROBOT_DISABLED.onFalse(Commands.runOnce(() ->
//                        CompletableFuture.runAsync(() -> REGISTERED_SUBSYSTEMS.forEach((subsystem) -> subsystem.setIdleMode(MotorProperties.IdleMode.BRAKE))))
//
//                .ignoringDisable(true));
//    }
//
//    public GenericSubsystem() {
//        GenericSubsystem.REGISTERED_SUBSYSTEMS.add(this);
//    }
//
//    private final SysIdRoutine routine = generateSysIdRoutine();
//
//    private static MotorProperties.IdleMode defaultDisableState = MotorProperties.IdleMode.COAST;
//    private static MotorProperties.IdleMode defaultEnableState = MotorProperties.IdleMode.BRAKE;
//
//    /**
//     * Creates a quasistatic (ramp up) command for characterizing the subsystem's mechanism.
//     *
//     * @param direction the direction in which to run the test
//     * @return the command
//     * @throws IllegalStateException if the {@link GenericSubsystem#getSysIdConfig()} function wasn't overridden or returns null
//     */
//    public final Command getQuasistaticCharacterizationCommand(SysIdRoutine.Direction direction) throws IllegalStateException {
//        if (routine == null)
//            throw new IllegalStateException("Subsystem " + getName() + " doesn't have a SysId routine!");
//        return routine.quasistatic(direction);
//    }
//
//    /**
//     * Creates a dynamic (constant "step up") command for characterizing the subsystem's mechanism.
//     *
//     * @param direction the direction in which to run the test
//     * @return the command
//     * @throws IllegalStateException if the {@link GenericSubsystem#getSysIdConfig()} function wasn't overridden or returns null
//     */
//    public final Command getDynamicCharacterizationCommand(SysIdRoutine.Direction direction) throws IllegalStateException {
//        if (routine == null)
//            throw new IllegalStateException("Subsystem " + getName() + " doesn't have a SysId routine!");
//
//        return routine.dynamic(direction);
//    }
//
//    public SysIdRoutine.Config getSysIdConfig() {
//        return null;
//    }
//
//    public void sysIdDrive(Measure<Voltage> voltageMeasure) {
//    }
//
//    public void sysIdUpdateLog(SysIdRoutineLog log) {
//    }
//
//    public abstract void setIdleMode(MotorProperties.IdleMode idleMode);
//
//    public abstract void stop();
//
//
//    private SysIdRoutine generateSysIdRoutine() {
//        if (getSysIdConfig() == null)
//            return null;
//
//        return new SysIdRoutine(
//                getSysIdConfig(),
//                new SysIdRoutine.Mechanism(
//                        this::sysIdDrive,
//                        this::sysIdUpdateLog,
//                        this,
//                        getName()
//                )
//        );
//    }
//}
