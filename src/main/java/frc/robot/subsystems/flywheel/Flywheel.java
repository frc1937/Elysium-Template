package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.lib.util.commands.ExecuteEndCommand;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.flywheel.real.RealFlywheelConstants;
import frc.robot.subsystems.flywheel.simulation.SimulatedFlywheelConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.flywheel.FlywheelConstants.SYSID_CONFIG;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO[] flywheels = generateFlywheels();

    private final SysIdRoutine routine;

    public Flywheel() {
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (voltage) -> setRawVoltage(0, voltage.in(Volts)),
                (SysIdRoutineLog log) -> sysIdLogFlywheel(0, log),
                this
        );

        routine = new SysIdRoutine(SYSID_CONFIG, mechanism);
    }

    public Command setFlywheelTarget(double velocityRotationsPerSecond) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTargetVelocity(velocityRotationsPerSecond),
                this::stop,
                this
        );
    }

    public Command sysIdQuastaticTest(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamicTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command setFlywheelTargetTangentialVelocity(double velocityMetersPerSecond) {
        return Commands.startEnd(
                () -> setFlywheelsTargetVelocity(velocityMetersPerSecond),
                this::stop,
                this
        );
    }

    public boolean hasReachedTarget() {
        for (FlywheelIO flywheel : flywheels) {
            if (!flywheel.hasReachedTarget()) {
                return false;
            }
        }

        return true;
    }

    @Override
    public void periodic() {
        for (FlywheelIO flywheel : flywheels) {
            flywheel.periodic();
        }
    }

    private void setRawVoltage(int flywheelIndex, double voltage) {
        flywheels[flywheelIndex].setRawVoltage(voltage);
    }

    private void sysIdLogFlywheel(int flywheelIndex, SysIdRoutineLog log) {
        FlywheelIO currentFlywheel = flywheels[flywheelIndex];

        log.motor(currentFlywheel.getName())
                .voltage(Volts.of(currentFlywheel.getVoltage()))
                .angularVelocity(RotationsPerSecond.of(currentFlywheel.getVelocityRotationsPerSecond())
                );
    }

    private void setFlywheelTangentialVelocity(double velocityMetersPerSecond) {
        for (FlywheelIO flywheel : flywheels) {
            flywheel.setTargetVelocity(Conversions.mpsToRps(velocityMetersPerSecond, flywheel.getFlywheelDiameter()));
        }
    }

    private void setFlywheelsTargetVelocity(double velocityRotationsPerSecond) {
        for (FlywheelIO flywheel : flywheels) {
            System.out.println("Flywheel #" + flywheel.getName());
            setFlywheelTargetVelocity(flywheel, velocityRotationsPerSecond);
        }
    }

    private void setFlywheelTargetVelocity(FlywheelIO flywheel, double velocityRotationsPerSecond) {
        System.out.println("NIGFlywheel #" + flywheel.getName());
        flywheel.setTargetVelocity(velocityRotationsPerSecond);
    }

    private void stop() {
        for (FlywheelIO flywheel : flywheels) {
            flywheel.stop();
            flywheel.setTargetVelocity(0);
        }
    }

    private FlywheelIO[] generateFlywheels() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REAL)
            return RealFlywheelConstants.getFlywheels();

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION)
            return SimulatedFlywheelConstants.getFlywheels();

        return FlywheelConstants.getFlywheels();
    }
}
