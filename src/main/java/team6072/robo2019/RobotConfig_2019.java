package team6072.robo2019;



public class RobotConfig_2019 {

    // constants (units in inches)
    public static double DRIVE_WHEEL_DIAMETER = 6.0;

    // Drive Talons
    public static int DRIVE_LEFT_MASTER = 16;
    public static int DRIVE_LEFT_SLAVE0 = 1;
    public static int DRIVE_LEFT_SLAVE1 = 2;

    public static int DRIVE_RIGHT_MASTER = 15;
    public static int DRIVE_RIGHT_SLAVE0 = 14;
    public static int DRIVE_RIGHT_SLAVE1 = 13;

    public static int DRIVE_GEAR_FWD_LO = 4; // DIO for the gear shifting solenoid
    public static int DRIVE_GEAR_REV_HI = 5;

    // ELEVATOR
    public static int ELEVATOR_MASTER = 4;
    public static int ELEVATOR_SLAVE = 5;

    // WRIST
    public static int WRIST_MASTER = 11;
    public static int WRIST_SLAVE = 10;


    // CLIMBER
    public static int CLIMBER_MASTER = 6;
    public static int CLIMBER_SLAVE = 7;


    // INTAKE
    public static int INTAKE_MASTER = 8;
    public static int INTAKE_SLAVE0 = 9;


    // PNEUMATICS
    public static int PCM_ID = 61;

    // Power Distribution Panel - NOTE FRC says needs to be 0 for 2018 WPILib
    public static int PDP_ID = 0;


    /**
     * There are two double solenoids controlled by the PCM solenoid 1 select open
     * or close if open, set sol 2 off
     *
     * solenoid 2 select close lo pressure or close high pressure
     */
    public static int INTAKE_SOL_1_FWD_OPEN = 0;
    public static int INTAKE_SOL_1_REV_CLOSE = 1;
    public static int INTAKE_SOL_2_FWD_LO = 2;
    public static int INTAKE_SOL_2_REV_HI = 3;

}