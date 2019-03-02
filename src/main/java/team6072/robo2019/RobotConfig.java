package team6072.robo2019;

public class RobotConfig {

    public static final boolean IS_ROBO_2019 = true;

    // constants (units in inches)
    public static double DRIVE_WHEEL_DIAMETER = 6.0;

    // DRIVE
    // Objective is to have the Talon LEDs flashing green when driving forward, and
    // sensor phase in sync with velocity - use the graphing in the Phoenix Tuner
    // plot to check
    public static final boolean DRIVE_LEFT_INVERT = true; // 2018 - false 2019 - true tested 2-19
    public static final boolean DRIVE_LEFT_SENSPHASE = false; // 2018 - true 2019 - false tested 2-19

    public static final boolean DRIVE_RIGHT_INVERT = false; // 2018 - true 2019 - false tested 2-19
    public static final boolean DRIVE_RIGHT_SENSPHASE = false; // 2018 - true 2019 - false tested 2-19

    public static final int DRIVE_LEFT_MASTER = 15;
    public static final int DRIVE_LEFT_SLAVE0 = 14;
    public static final int DRIVE_LEFT_SLAVE1 = 13; // only in 2019

    public static final int DRIVE_RIGHT_MASTER = 16;
    public static final int DRIVE_RIGHT_SLAVE0 = 1;
    public static final int DRIVE_RIGHT_SLAVE1 = 2; // only in 2019

    public static final int DRIVE_GEAR_FWD_LO = 4; // DIO for the gear shifting solenoid
    public static final int DRIVE_GEAR_REV_HI = 5;

    // ELEVATOR
    public static final boolean ELV_INVERT = false; // 2018 - false 2019- false tested 2-19
    public static final boolean ELV_SENSOR_PHASE = false; // 2018 false 2019 false tested 2019

    public static final int ELV_TICKS_PER_INCH = 371; // NOT MEASURED
    public static final double ELV_BASE_PERCENT_OUT = 0.0;

    public static final int ELEVATOR_MASTER = 4;
    public static final int ELEVATOR_SLAVE0 = 5; // only in 2019

    public static final int ELEVATOR_SWITCH_TOP = 1;
    public static final int ELEVATOR_SWITCH_BOT = 0;

    // WRIST
    public static final int WRIST_TICKS_PER_DEG = 11; // NOT MEASURED
    public static final boolean WRIST_SENSOR_PHASE = false;  // changed 2019-03-02 after wrist fix
    public static final boolean WRIST_INVERT = true;
    public static final double WRIST_BASE_PERCENT_OUT = 0.0;

    public static final int WRIST_MASTER = 11;
    public static final int WRIST_SLAVE0 = 10; // only in 2019

    public static final int WRIST_SWITCH_TOP = 3;
    public static final int WRIST_SWITCH_BOT = 2;

    // INTAKE
    public static final int INTAKE_TALON_LEFT = 2; // 2018
    public static final int INTAKE_TALON_RIGHT = 4; // 2018
    public static final int INTAKE_MASTER = 9; // 2019
    // public static final int INTAKE_SLAVE0 = 9; // 2019 - not needed?

    // CLIMBER
    public static final int CLIMBER_MASTER = 6; // 2019
    public static final int CLIMBER_SLAVE0 = 7; // 2019

    // Power Distribution Panel - NOTE FRC says needs to be 0 for 2018 WPILib
    public static final int PDP_ID = 0;

    // PNEUMATICS
    public static final int PCM_ID = 61;

    public static final int PCM_HATCH_EXTEND = 0;
    public static final int PCM_HATCH_RETRACT = 1;
    public static final int PCM_INTAKE_OPEN = 2;
    public static final int PCM_INTAKE_CLOSED = 3;
    public static final int PCM_DRIVE_HIGH = 4;
    public static final int PCM_DRIVE_LOW = 5;

}