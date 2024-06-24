package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private Command stopRumblingCommand = null;

    public enum Inputs {
        A(1), B(2), X(3), Y(4),
        LEFT_BUMPER(5), RIGHT_BUMPER(6),
        BACK(7), START(8);
        public final int id;

        Inputs(int id) {
            this.id = id;
        }
    }

    public enum Stick {
        LEFT_STICK(2), RIGHT_STICK(3);
        public final int value;

        Stick(int value) {
            this.value = value;
        }
    }
    public enum Axis {
        LEFT_X(0), RIGHT_X(4), LEFT_Y(1), RIGHT_Y(5);
        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum DPad {
        UP(0), UP_RIGHT(45), RIGHT(90), DOWN_RIGHT(135), DOWN(180), DOWN_LEFT(225), LEFT(270), UP_LEFT(315);
        public final int angle;

        DPad(int angle) {
            this.angle = angle;
        }
    }

    private final XboxController xboxController;
    private final int port;

    public Controller(int port) {
        this.port = port;
        this.xboxController = new XboxController(port);
    }

    /**
     * Returns a trigger that is active when the stick is pushed past 50%
     *
     * @param button - the button to create a trigger for, must be a stick
     * @return a trigger that is active when the stick is pushed past 50%
     */
    public Trigger getStick(Stick button) {
        return new Trigger(() -> xboxController.getRawAxis(button.value) > 0.5);
    }

    /**
     * Returns a trigger who is conditioned to the DPad
     *
     * @param padAngle - the angle of the DPad to create a trigger for
     * @return a trigger who is conditioned to the DPad
     */
    public Trigger getDPad(DPad padAngle) {
        return new Trigger(() -> xboxController.getPOV() == padAngle.angle);
    }

    /**
     * Returns a trigger that is active when the button is pressed
     *
     * @param button - the button to create a trigger for, must not be a stick
     * @return a trigger that is active when the button is pressed
     */
    public Trigger getButton(Inputs button) {
        return new JoystickButton(xboxController, button.id);
    }

    public double getRawAxis(Axis axis) {
        return DriverStation.getStickAxis(port, axis.value);
    }

    public void rumble(double intensity, double durationSeconds) {
        if (stopRumblingCommand != null) stopRumblingCommand.cancel();

        stopRumblingCommand = new WaitCommand(durationSeconds).andThen(() -> xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0));
        stopRumblingCommand.schedule();

        xboxController.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
    }

    public XboxController getXboxController() {
        return xboxController;
    }
}
