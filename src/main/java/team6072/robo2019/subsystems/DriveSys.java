package team6072.robo2019.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PIDController;

import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;

/**
 * Implement a drive subsystem for the 2019 robot Two motors per side, driving a
 * single shaft per side Each motor controlled by a CANTalon on the CAN bus
 *
 * As at 2018-02-10, the Talons have been given device IDs that match the PDP
 * port they are connected to See RobotConfig for details
 */
public class DriveSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(DriveSys.class.getName());

    private WPI_TalonSRX mLeft_Master;
    private WPI_TalonSRX mLeft_Slave0;
    private WPI_TalonSRX mRight_Master;
    private WPI_TalonSRX mRight_Slave0;

    ArrayList<TalonSRX> mMasterTalons = new ArrayList<TalonSRX>();

    private DifferentialDrive mRoboDrive;

    private static DriveSys mInstance;

    public static DriveSys getInstance() {
        if (mInstance == null) {
            mInstance = new DriveSys();
        }
        return mInstance;
    }


    // Objective is to have the Talon LEDs flashing green when driving forward, and
    // sensor phase in sync with velocity - use the graphing in the Phoenix Tuner plot to check
    private static boolean LEFT_INVERT = false;
    private static boolean LEFT_SENSPHASE = true;

    private static boolean RIGHT_INVERT = true;
    private static boolean RIGHT_SENSPHASE = true;

    /**
     * When configuring talon, the configXXX() methods have a timeout param.
     * Recommended value when config during init is 10mSec, because each call will
     * wait to ensure value set correctly During run time, want the value to be 0 to
     * avoid blocking main thread
     */
    private DriveSys() {
        mLog.info("DriveSys ctor  ----------------------------------------------");

        try {
            mLeft_Master = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_MASTER);
            mLeft_Master.setSafetyEnabled(false);
            // SetInverted is added to decide if motor should spin clockwise or counter
            // clockwise when told to move positive/forward (green LEDs)
            // This will invert the hbridge output but NOT the LEDs.
            mLeft_Master.setInverted(LEFT_INVERT);
            mLeft_Master.setSensorPhase(LEFT_SENSPHASE);

            mLeft_Master.configOpenloopRamp(0.1, 10);
            mLeft_Master.setNeutralMode(NeutralMode.Brake);
            // nominal outputs can be selected to ensure that any non-zero requested motor
            // output gets promoted to a minimum output. For example, if the nominal forward
            // is set to +0.10 (+10%), then any motor request within (0%, +10%) will be
            // promoted to +10% assuming request is beyond the neutral dead band.
            mLeft_Master.configNominalOutputForward(0.1, 10);
            mLeft_Master.configNominalOutputReverse(-0.1, 10);

            // Current Limiting: If enabled, Talon SRX will monitor the supply-current
            // looking for a conditions where current has exceeded the Peak Current for at
            // least Peak Time. If detected, output is reduced until current measurement is
            // at or under Continuous Current.
            mLeft_Master.configPeakCurrentLimit(100, 10);
            mLeft_Master.configPeakCurrentDuration(100);
            mLeft_Master.configContinuousCurrentLimit(50, 10);
            mLeft_Master.enableCurrentLimit(true);

            mLeft_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_SLAVE0);
            mLeft_Slave0.follow(mLeft_Master, FollowerType.PercentOutput);
            mLeft_Slave0.setInverted(InvertType.FollowMaster);

            mRight_Master = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_MASTER);
            mRight_Master.setInverted(RIGHT_INVERT);
            mRight_Master.setSensorPhase(RIGHT_SENSPHASE);

            mRight_Master.setSafetyEnabled(false);
            mRight_Master.configOpenloopRamp(0.1, 10);
            mRight_Master.setNeutralMode(NeutralMode.Brake);

            mRight_Master.configPeakCurrentLimit(100, 10);
            mRight_Master.configPeakCurrentDuration(100);
            mRight_Master.configContinuousCurrentLimit(50, 10);
            mRight_Master.enableCurrentLimit(true);

            mRight_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_SLAVE0);
            mRight_Slave0.follow(mRight_Master, FollowerType.PercentOutput);
            mRight_Slave0.setInverted(InvertType.FollowMaster);

            mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);
            /*
             * WPI drivetrain classes assume left and right are opposite by default. 
             * Call this so we can apply + to both sides when moving forward. DO NOT CHANGE
             */
            mRoboDrive.setRightSideInverted(false);

            mMasterTalons.add(mRight_Master);
            mMasterTalons.add(mLeft_Master);

            setSensorStartPosn();

            mNavX = NavXSys.getInstance().getNavX();
            initYawPID();
            initDrivePID();
            mLog.info("DriveSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "DriveSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }

    /**
     * Each subsystem may, but is not required to, have a default command which is
     * scheduled whenever the subsystem is idle (the command currently requiring the
     * system completes). The most common example of a default command is a command
     * for the drivetrain that implements the normal joystick control. This command
     * may be interrupted by other commands for specific maneuvers ("precision
     * mode", automatic alignment/targeting, etc.) but after any command requiring
     * the drivetrain completes the joystick command would be scheduled again.
     */
    public void initDefaultCommand() {
        mLog.info("DriveSys: init default command");
    }


    /**
     * Utility method to sleep for a period if we need to
     */
    private void sleep(int milliSecs) {
        try {
            Thread.sleep(milliSecs);
        } catch (Exception ex) {
        }
    }

    public String logSensors() {
        int leftPW = mLeft_Master.getSensorCollection().getPulseWidthPosition();
        int rightPW = mRight_Master.getSensorCollection().getPulseWidthPosition();
        int leftQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
        int rightQuad = mRight_Master.getSensorCollection().getQuadraturePosition();
        return String.format("DS.sensors:  leftPW: %5d  leftQuad: %5d   rightPW: %5d   rightQuad: %5d", leftPW, leftQuad, rightPW, rightQuad);
    }


    /**
     * Set the quad posn to same as PW posn. Left motor sensor goes -ve when driving
     * forward This should only be called from Robot.Init because of the time delays
     */
    public void setSensorStartPosn() {
        mLog.debug("DS.setSensStart  beg: " + logSensors());
        mLeft_Master.getSensorCollection().setPulseWidthPosition(0, 10);
        mRight_Master.getSensorCollection().setPulseWidthPosition(0, 10);
        int leftPosn = mLeft_Master.getSensorCollection().getPulseWidthPosition();
        int rightPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();

        /* mask out overflows, keep bottom 12 bits */
        int leftAbsPosition = leftPosn & 0xFFF;
        if (LEFT_SENSPHASE) {
            leftAbsPosition *= -1;
        }
        if (LEFT_INVERT) {
            leftAbsPosition *= -1;
        }
        int rightAbsPosn = rightPosn & 0xFFF;
        if (RIGHT_SENSPHASE) {
            rightAbsPosn *= -1;
        }
        if (RIGHT_INVERT) {
            rightAbsPosn *= -1;
        }
        mLeft_Master.setSelectedSensorPosition(leftAbsPosition, 0, 30);
        mRight_Master.setSelectedSensorPosition(rightAbsPosn, 0, 30);
        // setSelected takes time so wait for it to get accurate print
        mLog.debug("DS.setSensStart  end: " + logSensors());
        sleep(100);
        mLog.debug("DS.setSensStart  end2: " + logSensors());
        sleep(100);
        mLog.debug("DS.setSensStart  end3: " + logSensors());
    }


    public int getLeftSensPosn() {
        return mLeft_Master.getSensorCollection().getPulseWidthPosition();
    }


    /**
     * Drive the system
     * @param mag   -  +ve for forward
     * @param yaw   -  +ve for right
     */
    public void arcadeDrive(double mag, double yaw) {
        yaw = yaw * 0.8;        // reduce sensitivity on turn
        mRoboDrive.arcadeDrive(mag, yaw, true);
    }


    //  ---------------------------  Drive a specified distance ------------------------------------


    private int mTargetDist = 0;            // distance to travel in ticks
    private int mStartPosn = 0;             // start position in Talon ticks
    private int mTargPosn = 0;              // mStartPosn + mTargetDist
    private boolean mHitTarg = false;       // set true when we hit target
    private int mMoveDistLoopCnt;           // used for debug output

    /**
     * Tell the drive system that we want to drive N feet. Assume that the yaw PID
     * has been initialized already - we may need to set up here later Need to: log
     * our start position calculate how many ticks we need to travel
     * 
     * @param distInFeet
     */
    public void initDriveDist(double distInFeet) {
        double circumferenceInFeet = Math.PI * 0.5;
        mTargetDist = (int) ((distInFeet / (circumferenceInFeet)) * 4096);
        mStartPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();
        mTargPosn = mStartPosn + mTargetDist;
        //System.out.println("InitDd:  " + distInFeet + "  " + mTargetDist + "  " + mStartPosn + "  " + mTargPosn);
        mLog.debug(
                "DS.initDriveDist:  distInFeet: %.3f   mTargdist:  %d    mStartPos: %d  mTargPosn: %d\n",
                distInFeet, mTargetDist, mStartPosn, mTargPosn);
        mLog.debug("DS.initDriveDist: current yaw: %.3f", NavXSys.getInstance().getYawHeading());
        mHitTarg = false;
        mMoveDistLoopCnt = 0;
        mYawPID.setSetpoint(NavXSys.getInstance().getYawHeading());
        mYawPID.enable();
        mDrivePID.setSetpoint(mTargPosn);
        mDrivePID.enable();
    }


    /**
     * While we have not completed driving, move forward Need to: check current
     * position decide if have driven far enough if not, keep driving if yes, stop
     */
    public void driveDist() {
        int curPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();
        double mag = mDrivePIDOut.getVal();
        double yaw = mYawPIDOut.getVal();
        if (mMoveDistLoopCnt++ % 5 == 0) {
            mLog.debug("DS.driveDist: start: %d   cur: %d   targ: %d   mag: %.3f  yaw: %.3f  \r\n",
                    mStartPosn, curPosn, mTargPosn, mag, yaw);
        }
        mRoboDrive.arcadeDrive(mag, yaw, false);
        mHitTarg = mDrivePID.onTarget();
    }

    
    /**
     * Called by external system to find out if we have completed driving distance
     * 
     * @return true if have completed driving
     */
    public boolean isDriveDistComplete() {
        if (mDrivePID.onTarget()) {
            int curPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();
            mLog.debug("DS.driveDist complete: targ: %d   cur: %d  ---------------------------------", mTargPosn,
                    curPosn);
            mLog.debug(logSensors());
            sleep(100);
            mLog.debug(logSensors());
            return true;
        }
        return false;
    }



    // ---------------- Drive distance PID -----------------------------------------

    // resources for understanding PID
    // http://blog.opticontrols.com/archives/344 -- good intro to PID
    // https://www.controleng.com/articles/feed-forwards-augment-pid-control/ --
    // what the feed forward term does

    // magic values used to initialize the PID controller
    static final double kF_drive = 2.0; // feed forward - tries to estimate target and set motor to correct value
    static final double kP_drive = 1.0; // specify the proportional (fixed) response to error
    static final double kI_drive = 1.0; // specify response based on how big error is - larger error means bigger
                                        // response
    static final double kD_drive = 0.00; // rarely used - specify response based on how fast error is changing

    // calculate allowed dist error in inches
    private static final int ALLOWED_DISTERR = (int) (6 / (Math.PI * 6) * 4096);

    private PIDController mDrivePID;
    private PIDOutReceiver mDrivePIDOut;
    private PIDSourceTalonPW mTalonPIDSource;

    
    private void initDrivePID() {
        mLog.info("DS.initDrivePID:  ");
        mDrivePIDOut = new PIDOutReceiver();
        mTalonPIDSource = new PIDSourceTalonPW(mRight_Master);
        mDrivePID = new PIDController(kP_drive, kI_drive, kD_drive, kF_drive, mTalonPIDSource, mDrivePIDOut);
        mDrivePID.setName("DS.DrivePID");
        // mDrivePID.setInputRange(-180.0f, 180.0f);
        mDrivePID.setOutputRange(-0.8, 0.8);
        // Makes PIDController.onTarget() return True when PIDInput is within the
        // Setpoint +/- the absolute tolerance.
        mDrivePID.setAbsoluteTolerance(ALLOWED_DISTERR);
        // Treats the input ranges as the same, continuous point rather than two
        // boundaries, so it can calculate shorter routes.
        // For example, in a Drive, 0 and 360 are the same point, and should be
        // continuous. Needs setInputRanges.
        mDrivePID.setContinuous(false);
    }




    // ------------------ NavX PID for yaw ------------------------------------

    static final double kF_yaw = 0.00;
    static final double kP_yaw = 0.03;
    static final double kI_yaw = 0.00;
    static final double kD_yaw = 0.00;
    /* This tuning parameter indicates how close to "on target" the */
    /* PID Controller will attempt to get. */
    static final double kToleranceDegrees = 2.0f;

    private AHRS mNavX;
    private PIDController mYawPID;
    private PIDOutReceiver mYawPIDOut;

    private void initYawPID() {
        mLog.info("DS.initYawPID:  AHRS.SrcType: " + mNavX.getPIDSourceType().name());
        mYawPIDOut = new PIDOutReceiver();
        mYawPID = new PIDController(kP_yaw, kI_yaw, kD_yaw, kF_yaw, mNavX, mYawPIDOut);
        mYawPID.setName("DS.YawPID");
        mYawPID.setInputRange(-180.0f, 180.0f);
        mYawPID.setOutputRange(-1.0, 1.0);
        // Makes PIDController.onTarget() return True when PIDInput is within the
        // Setpoint +/- the absolute tolerance.
        mYawPID.setAbsoluteTolerance(kToleranceDegrees);
        // Treats the input ranges as the same, continuous point rather than two
        // boundaries, so it can calculate shorter routes.
        // For example, in a gyro, 0 and 360 are the same point, and should be
        // continuous. Needs setInputRanges.
        mYawPID.setContinuous(true);
        // mYawPID.setSetpoint(0);
        // mYawPID.enable();
    }



}