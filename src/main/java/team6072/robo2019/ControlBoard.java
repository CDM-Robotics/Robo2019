package team6072.robo2019;

import java.util.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.commands.drive.*;
import team6072.robo2019.commands.elevator.*;
import team6072.robo2019.commands.wrist.*;
import team6072.robo2019.commands.intake.*;
import team6072.robo2019.commands.objectives.ObjectiveCmd;
import team6072.robo2019.commands.pneumatics.*;
import team6072.robo2019.commands.objectives.Objective;
import team6072.robo2019.subsystems.WristSys.WristTarget;
import team6072.robo2019.commands.LED.LEDKillCmd;
import team6072.robo2019.commands.LED.LEDLightsOnCmd;
import team6072.robo2019.commands.RoboLord.RoboLordCmd;
import team6072.robo2019.commands.climber.*;
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
    // Y-axis - forward and back
    // X-axis - left and right
    // Z-axis - twist
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
    public static int EXTREME_BUT_10 = 10; // not working?
    public static int EXTREME_BUT_11 = 11; // not working?
    public static int EXTREME_BUT_12 = 12;
    // left panel buttons
    public static int LEFTPANEL_BUT_1 = 7;
    public static int LEFTPANEL_BUT_2 = 10;
    public static int LEFTPANEL_BUT_3 = 12;
    public static int LEFTPANEL_BUT_4 = 8;
    public static int LEFTPANEL_BUT_5 = 9;
    public static int LEFTPANEL_BUT_6 = 11;
    // right panel buttons
    public static int RIGHT_PANEL_BUT_1 = 7;
    public static int RIGHT_PANEL_BUT_2 = 10;
    public static int RIGHT_PANEL_BUT_3 = 11;
    public static int RIGHT_PANEL_BUT_4 = 8;
    public static int RIGHT_PANEL_BUT_5 = 9;
    public static int RIGHT_PANEL_BUT_6 = 12;

    private ArrayList<Button> mButtonList;

    // drive stick is used for driving robot
    private static int DRIVE_USB_PORT = 0;
    public Joystick mDriveStick;

    // control stick is used for elevator, intake
    private static int CONTROL_USB_PORT = 1;
    public Joystick mControlStick;

    // left panel used for cargo ship calculations
    private static int LPANEL_USB_PORT = 2;
    public Joystick mLeftPanel;

    // right panel used for rocket calculations
    private static int RPANEL_USB_PORT = 3;
    public Joystick mRightPanel;

    private static ControlBoard mInstance;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private ControlBoard() {

        mButtonList = new ArrayList<Button>();

        mDriveStick = new Joystick(DRIVE_USB_PORT);
        mControlStick = new Joystick(CONTROL_USB_PORT);
        mLeftPanel = new Joystick(LPANEL_USB_PORT);
        mRightPanel = new Joystick(RPANEL_USB_PORT);

        // Drive Stick Commands --------------------------------------

        MapCmdToBut(mDriveStick, EXTREME_BUT_LEFT_BOT, new LEDLightsOnCmd(.5), null);
        MapCmdToBut(mDriveStick, EXTREME_BUT_RIGHT_TOP, new LEDKillCmd(), null);  // DEBUG


        // Control Stick Commands ------------------------------------

        // MapCmdToBut(mDriveStick, EXTREME_BUT_TRIGGER, new ElvMoveUpSlow(), null);

        MapCmdToBut(mControlStick, EXTREME_BUT_LEFT_TOP, new ElvMoveUpCmd(), new ElvHoldCmd()); //new ElvHoldPIDCmd()
        MapCmdToBut(mControlStick, EXTREME_BUT_LEFT_BOT, new ElvMoveDownCmd(), new ElvHoldCmd()); //new ElvHoldPIDCmd()

        MapCmdToBut(mControlStick, EXTREME_BUT_RIGHT_TOP, new WristExtendCmd(), new WristHoldPIDCmd());  // new WristHoldPIDCmd());
        MapCmdToBut(mControlStick, EXTREME_BUT_RIGHT_BOT, new WristRetractCmd(), new WristHoldPIDCmd()); // new WristHoldPIDCmd());

        MapCmdToBut(mControlStick, EXTREME_BUT_TRIGGER, new IntakeWheelsInCmd(), new IntakeWheelsStopCmd());
        MapCmdToBut(mControlStick, EXTREME_BUT_THUMB, new IntakeWheelsOutCmd(), new IntakeWheelsStopCmd());

        MapCmdToPovBut(mControlStick, PovAngle.Deg_000, new FlowerOpenCmd(), null);
        MapCmdToPovBut(mControlStick, PovAngle.Deg_180, new FlowerCloseCmd(), null);

        MapCmdToPovBut(mControlStick, PovAngle.Deg_090, new FlowerWristExtendCmd(), null);
        MapCmdToPovBut(mControlStick, PovAngle.Deg_270, new FlowerWristRetractCmd(), null);

        MapCmdToBut(mControlStick, EXTREME_BUT_7, new ElvKillWatchDogCmd(), null);
        MapCmdToBut(mControlStick, EXTREME_BUT_8, new ElvReviveWatchDogCmd(), null);

        // MapCmdToBut(mControlStick, EXTREME_BUT_9, new WristStopCmd(), null);

        // right panel -------------------------------------------------

        // MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_4, new RoboLordCmd(), null);

        // MapCmdToBut(mDriveStick, EXTREME_BUT_TRIGGER, new AlignmentTurnCmd(Objective.TargetYaw.LEFT_BALL), null);

        MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_1, new ObjectiveCmd(Objective.ElvTarget.RocketCargoHi), null);
        MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_2, new ObjectiveCmd(Objective.ElvTarget.RocketCargoMid), null);
        MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_3, new ObjectiveCmd(Objective.ElvTarget.RocketCargoLo), null);
        MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_4, new ObjectiveCmd(Objective.ElvTarget.RocketHatchHi), null);
        MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_5, new ObjectiveCmd(Objective.ElvTarget.RocketHatchMid), null);
        MapCmdToBut(mRightPanel, RIGHT_PANEL_BUT_6, new ObjectiveCmd(Objective.ElvTarget.RocketHatchLo), null);

        // left panel -----------------------------------------------------

        MapCmdToBut(mLeftPanel, LEFTPANEL_BUT_1, new ObjectiveCmd(Objective.ElvTarget.HatchPickUp), null);
        MapCmdToBut(mLeftPanel, LEFTPANEL_BUT_4, new ObjectiveCmd(Objective.ElvTarget.CargoshipCargo), null);
        MapCmdToBut(mLeftPanel, LEFTPANEL_BUT_5, new ObjectiveCmd(Objective.ElvTarget.CargoshipHatch), null);

    }

    /**
     * Create a new Joystick button with the commands attached to it.
     * 
     * @param stick
     * @param button
     * @param pressCmd   - command for when button pressed
     * @param releaseCmd - command for when button released
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
    
    public enum PovAngle {
        Deg_000(0), Deg_045(45), Deg_090(90), Deg_135(135), Deg_180(180), Deg_225(225), Deg_270(270), Deg_315(315);

        private int mAngle;

        PovAngle(int angle) {
            mAngle = angle;
        }

        public int getAngle() {
            return mAngle;
        }
    }

    /**
     * See PovButton here:
     * http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/buttons/POVButton.html
     * 
     * PovButton is the small rotating button on top of the joystick
     */
    private void MapCmdToPovBut(Joystick stick, PovAngle angle, Command pressCmd, Command releaseCmd) {
        POVButton but = new POVButton(stick, angle.getAngle());
        if (pressCmd != null) {
            but.whenPressed(pressCmd);
        }
        if (releaseCmd != null) {
            but.whenReleased(releaseCmd);
        }
        mButtonList.add(but);
    }

}