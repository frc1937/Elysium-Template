package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTunableNumber;

import static frc.robot.RobotContainer.ARM;

public class StaticFrictionCharacterization extends Command {
    LoggedTunableNumber num = new LoggedTunableNumber("StaticFrictionCharacterization", 0);
    double rampingVoltage = 0.11;

    public StaticFrictionCharacterization() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ARM.UNSAFE_setVoltage(rampingVoltage);

        if (ARM.getVelocity() != 0) {
            System.out.println("MOVED AT VOLTAGE OF " + rampingVoltage + " WAS " + ARM.getVelocity());
        } else {
            rampingVoltage += 0.001;
            System.out.println("ARM AIT MOVING speed: " + rampingVoltage);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
