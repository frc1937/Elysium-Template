package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.commands.ExecuteEndCommand;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.SYSID_CONFIG;

public class Flywheels extends SubsystemBase {
    private final FlywheelsInputsAutoLogged flywheelsInputs = new FlywheelsInputsAutoLogged();
    private final FlywheelsIO flywheels = FlywheelsIO.generateIO();

    private final FlywheelsConstants constants = FlywheelsConstants.generateConstants();
    private final SingleFlywheelIO[] singleFlywheels = getFlywheels();

    private final SysIdRoutine routine;

    public Flywheels() {
        int flywheelToSysID = 0;

        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (voltage) -> setRawVoltage(flywheelToSysID, voltage.in(Volts)),
                (SysIdRoutineLog log) -> sysIdLogFlywheel(flywheelToSysID, log),
                this
        );

        routine = new SysIdRoutine(SYSID_CONFIG, mechanism);
    }

    public Command setFlywheelsTargetVelocity(double targetRPS) {
        return new ExecuteEndCommand(
                () -> setTargetVelocity(targetRPS),
                this::stop,
                this
        );
    }

    public Command setFlywheelsTangentialVelocity(double velocityMetersPerSecond) {
        return new ExecuteEndCommand(
                () -> setTargetTangentialVelocity(velocityMetersPerSecond),
                this::stop,
                this
        );
    }

    @Override
    public void periodic() {
        flywheels.refreshInputs(flywheelsInputs);
        Logger.processInputs("Flywheels", flywheelsInputs);

        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.periodic();
    }

    public boolean hasReachedTarget() {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels) {
            if (!currentFlywheel.hasReachedTarget())
                return false;
        }

        return true;
    }

    public Command sysIdQuastaticTest(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamicTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    private void stop() {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.stop();
    }

    private void setTargetVelocity(double targetRPS) {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.setTargetVelocityRPS(targetRPS);
    }

    private void setTargetTangentialVelocity(double velocityMetersPerSecond) {
        for (SingleFlywheelIO currentFlywheel : singleFlywheels)
            currentFlywheel.setTargetTangentialVelocity(velocityMetersPerSecond);
    }

    private void setRawVoltage(int index, double voltage) {
        singleFlywheels[index].setRawVoltage(voltage);
    }

    private void sysIdLogFlywheel(int flywheelIndex, SysIdRoutineLog log) {
        SingleFlywheelIO currentFlywheel = singleFlywheels[flywheelIndex];

        log.motor(currentFlywheel.getName())
                .voltage(Volts.of(currentFlywheel.getVoltage()))
                .angularVelocity(RotationsPerSecond.of(currentFlywheel.getVelocityRotationsPerSecond()));
    }

    private SingleFlywheelIO[] getFlywheels() {
        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new SingleFlywheelIO[]{
                    new SingleFlywheelIO("Left"),
                    new SingleFlywheelIO("Right")
            };
        }

        return constants.getFlywheels().get();
    }
}
