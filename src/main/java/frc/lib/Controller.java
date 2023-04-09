package frc.lib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Joystick wrapper because ~~it was pissing me off~~ we need a cleaner RobotContainer.
 * <p>The methods in this class return triggers. To get boolean values, use {@link Trigger#getAsBoolean()} </p>
 */
public class Controller {
    private final Joystick joystick;

    private final Trigger start;
    private final Trigger back;

    private final Trigger leftStick;
    private final Trigger rightStick;

    private final Trigger leftBumper;
    private final Trigger rightBumper;

    private final Trigger leftTrigger;
    private final Trigger rightTrigger;

    private final Trigger a;
    private final Trigger b;
    private final Trigger x;
    private final Trigger y;

    private final Trigger up;
    private final Trigger down;
    private final Trigger left;
    private final Trigger right;

    private final Trigger upLeft;
    private final Trigger upRight;
    private final Trigger downLeft;
    private final Trigger downRight;

    public Controller(int port) {
        joystick = new Joystick(port);

        start = new JoystickButton(joystick, XboxController.Button.kStart.value);
        back = new JoystickButton(joystick, XboxController.Button.kBack.value);

        leftStick = new JoystickButton(joystick, XboxController.Button.kLeftStick.value);
        rightStick = new JoystickButton(joystick, XboxController.Button.kRightStick.value);

        leftBumper = new JoystickButton(joystick, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(joystick, XboxController.Button.kRightBumper.value);

        leftTrigger = new Trigger(() -> joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5);
        rightTrigger = new Trigger(() -> joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5);
        
        a = new JoystickButton(joystick, XboxController.Button.kA.value);
        b = new JoystickButton(joystick, XboxController.Button.kB.value);
        x = new JoystickButton(joystick, XboxController.Button.kX.value);
        y = new JoystickButton(joystick, XboxController.Button.kY.value);

        up = new Trigger(() -> joystick.getPOV() == 0);
        down = new Trigger(() -> joystick.getPOV() == 180);
        left = new Trigger(() -> joystick.getPOV() == 270);
        right = new Trigger(() -> joystick.getPOV() == 90);

        upLeft = new Trigger(() -> joystick.getPOV() == 315);
        upRight = new Trigger(() -> joystick.getPOV() == 45);
        downLeft = new Trigger(() -> joystick.getPOV() == 135);
        downRight = new Trigger(() -> joystick.getPOV() == 225);
    }

    public Joystick getJoystick() {
        return joystick;
    }

    public double getLeftX() {
        return joystick.getRawAxis(XboxController.Axis.kLeftX.value);
    }
    
    public double getLeftY() {
        return joystick.getRawAxis(XboxController.Axis.kLeftY.value);
    }
    
    public double getRightX() {
        return joystick.getRawAxis(XboxController.Axis.kRightX.value);
    }
    
    public double getRightY() {
        return joystick.getRawAxis(XboxController.Axis.kRightY.value);
    }

    public Trigger getStart() {
        return start;
    }

    public Trigger getBack() {
        return back;
    }

    public Trigger getLeftStick() {
        return leftStick;
    }

    public Trigger getRightStick() {
        return rightStick;
    }

    public Trigger getLeftBumper() {
        return leftBumper;
    }

    public Trigger getRightBumper() {
        return rightBumper;
    }

    public Trigger getLeftTrigger() {
        return leftTrigger;
    }

    public double getLeftTriggerAsAxis() {
        return joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    }

    public Trigger getRightTrigger() {
        return rightTrigger;
    }

    public double getRightTriggerAsAxis() {
        return joystick.getRawAxis(XboxController.Axis.kRightTrigger.value);
    }

    public Trigger getA() {
        return a;
    }

    public Trigger getB() {
        return b;
    }

    public Trigger getX() {
        return x;
    }

    public Trigger getY() {
        return y;
    }

    public Trigger getUp() {
        return up;
    }

    public Trigger getDown() {
        return down;
    }
    
    public Trigger getLeft() {
        return left;
    }
    
    public Trigger getRight() {
        return right;
    }
    
    public Trigger getUpLeft() {
        return upLeft;
    }
    
    public Trigger getUpRight() {
        return upRight;
    }
    
    public Trigger getDownLeft() {
        return downLeft;
    }
    
    public Trigger getDownRight() {
        return downRight;
    }
}
