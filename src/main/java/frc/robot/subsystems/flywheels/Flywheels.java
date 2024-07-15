package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.util.commands.ExecuteEndCommand;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.flywheels;

public class Flywheels extends GenericSubsystem {
    private final int flywheelIndexToLog = 0;

    public Command setTargetVelocity(double velocityRPS) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTargetVelocity(velocityRPS),
                this::stop,
                this
        );
    }

    public Command setTargetTangentialVelocity(double velocityMPS) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTangentialVelocity(velocityMPS),
                this::stop,
                this
        );
    }

    @Override
    public void periodic() {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.periodic();
        }
    }

    @Override
    public void sysIdDrive(double voltage) {
        flywheels[0].setVoltage(voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("Flywheel " + flywheelIndexToLog)
                .voltage(Volts.of(flywheels[flywheelIndexToLog].getVoltage()))
                .angularPosition(Rotations.of(flywheels[flywheelIndexToLog].getPosition()))
                .angularVelocity(RotationsPerSecond.of(flywheels[flywheelIndexToLog].getVelocity()));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return super.getSysIdConfig();
    }

    public boolean hasReachedTarget() {
        for (SingleFlywheel flywheel : flywheels) {
            if (!flywheel.hasReachedTarget()) {
                return false;
            }
        }
        return true;
    }

    public void stop() {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.stop();
        }
    }

    public void setFlywheelsTangentialVelocity(double targetMPS) {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.setTargetTangentialVelocity(targetMPS);
        }
    }

    public void setFlywheelsTargetVelocity(double targetRPS) {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.setTargetVelocity(targetRPS);
        }
    }
}
