package team6072.robo2019;

import java.util.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;


/**
 * ControlBoard holds the code for interacting with the
 */
public class ControlBoard {

    // logitech gamepad buttons
    public static int LOGITECH_BUT_A = 1;
    public static int LOGITECH_BUT_B = 2;
    public static int LOGITECH_BUT_X = 3;
    public static int LOGITECH_BUT_Y = 4;
    public static int LOGITECH_BUT_LEFT = 5;
    public static int LOGITECH_BUT_RIGHT = 6;
    
    // extreme buttons
    // Y-axis  -  forward and back
    // X-axis  -  left and right
    // Z-axis  -  twist
    // hub is POV?
    public static int EXTREME_BUT_TRIGGER = 1;
    public static int EXTREME_BUT_THUMB = 2;
    public static int EXTREME_BUT_LEFT_TOP = 5;
    public static int EXTREME_BUT_LEFT_BOT = 3;
    public static int EXTREME_BUT_RIGHT_TOP = 6;
    public static int EXTREME_BUT_RIGHT_BOT = 4;
    public static int EXTREME_BUT_7 = 7;
    public static int EXTREME_BUT_8 = 8;
    public static int EXTREME_BUT_9 = 9;
    public static int EXTREME_BUT_10 = 10;      // not working?
    public static int EXTREME_BUT_11 = 11;      // not working?
    public static int EXTREME_BUT_12 = 12;


    private ArrayList<JoystickButton> mButtonList;



    // drive stick is used for driving robot
    private static int DRIVE_USB_PORT = 0;
    public Joystick mDriveStick;


    // control stick is used for elevator, intake
    private static int CONTROL_USB_PORT = 1;
    public Joystick mControlStick;


    private static ControlBoard mInstance;
    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }




    private ControlBoard () {

        mButtonList = new ArrayList<JoystickButton>();

        mDriveStick = new Joystick(DRIVE_USB_PORT);
        //mControlStick = new Joystick(CONTROL_USB_PORT);

        //MapCmdToBut(mDriveStick, EXTREME_BUT_7, new DriveDistCmd(4), null);
    }


    /**
     * Create a new Joystick button with the commands attached to it.
     * @param stick
     * @param button
     * @param pressCmd      - command for when button pressed
     * @param releaseCmd    - command for when button released
     */
    private void MapCmdToBut(Joystick stick, int button, Command pressCmd, Command releaseCmd) {
        JoystickButton but = new JoystickButton(stick, button);
        if (pressCmd != null) {
            but.whenPressed(pressCmd);
        }
        if (releaseCmd != null) {
            but.whenReleased(releaseCmd);
        }
        mButtonList.add(but);
    }


}