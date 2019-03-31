package team6072.robo2019;

public class RobotConfig {

    public static final boolean IS_ROBO_2019 = false;

    // constants (units in inches)
    public static double DRIVE_WHEEL_DIAMETER = 6.0;

    // Drive Talons
    public static final boolean DRIVE_LEFT_INVERT = false; // 2018 - false
    public static final boolean DRIVE_LEFT_SENSPHASE = true; // 2018 - true

    public static final boolean DRIVE_RIGHT_INVERT = true; // 2018 - true
    public static final boolean DRIVE_RIGHT_SENSPHASE = true; // 2018 - true

    public static final int DRIVE_LEFT_MASTER = 15;
    public static final int DRIVE_LEFT_SLAVE0 = 14;
    public static final int DRIVE_LEFT_SLAVE1 = -1; // only in 2019

    public static final int DRIVE_RIGHT_MASTER = 30;
    public static final int DRIVE_RIGHT_SLAVE0 = 1;
    public static final int DRIVE_RIGHT_SLAVE1 = -1; // only in 2019

    public static final int DRIVE_GEAR_FWD_LO = 4; // DIO for the gear shifting solenoid
    public static final int DRIVE_GEAR_REV_HI = 5;

    // ELEVATOR
    public static final int ELV_TICKS_PER_INCH = 370; // MEASURED  2019-03-14
    public static final double ELV_BASE_PERCENT_OUT = 0.19;

    public static final boolean ELV_INVERT = false; // 2018 - false
    public static final boolean ELV_SENSOR_PHASE = false; // 2018 false

    public static final int ELEVATOR_MASTER = 12;
    public static final int ELEVATOR_SLAVE0 = -1; // only in 2019

    public static final int ELEVATOR_SWITCH_TOP = 1;
    public static final int ELEVATOR_SWITCH_BOT = 0;

    // WRIST
    public static final int WRIST_TICKS_PER_DEG = 200; // NOT MEASURED
    public static final boolean WRIST_SENSOR_PHASE = false; // SGT to get working on Mitch 2018
    public static final boolean WRIST_INVERT = true;
    public static final double WRIST_BASE_PERCENT_OUT = 0.0;

    public static final int WRIST_MASTER = 11;
    public static final int WRIST_SLAVE0 = -1; // only in 2019

    public static final int WRIST_SWITCH_TOP = 3;
    public static final int WRIST_SWITCH_BOT = 2;

    // INTAKE
    public static final int INTAKE_MASTER = 9; // 2019
    public static final int INTAKE_SLAVE0 = -1; // 2019

    // Power Distribution Panel - NOTE FRC says needs to be 0 for 2018 WPILib
    public static final int PDP_ID = 0;

    // PNEUMATICS
    public static final int PCM_ID = 61;

    /**
     * There are two double solenoids controlled by the PCM solenoid 1 select open
     * or close if open, set sol 2 off
     *
     * solenoid 2 select close lo pressure or close high pressure
     */
    public static final int INTAKE_SOL_1_FWD_OPEN = 0;
    public static final int INTAKE_SOL_1_REV_CLOSE = 1;
    public static final int INTAKE_SOL_2_FWD_LO = 2;
    public static final int INTAKE_SOL_2_REV_HI = 3;

    // PNEUMATICS for 2019 - just here to let code compile

    public static final int PCM_HATCH_EXTEND = 0;
    public static final int PCM_HATCH_RETRACT = 1;
    public static final int PCM_INTAKE_OPEN = 2;
    public static final int PCM_INTAKE_CLOSED = 3;
    public static final int PCM_DRIVE_HIGH = 4;
    public static final int PCM_DRIVE_LOW = 5;

    //CLIMBER for 2019

    public static final Double CLIMB_BASE_PERCENT_OUT = 0.0;
}