
package team6072.robo2019.subsystems;

import java.util.TimerTask;
import java.util.Timer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.commands.objectives.Objective;
import team6072.robo2019.logging.*;
import team6072.robo2019.pid.IPIDExecOnTarget;
import team6072.robo2019.pid.TTPIDController;

/**
 * Add your docs here.
 */
public class ElevatorSys extends Subsystem implements IPIDExecOnTarget  {

    private static final LogWrapper mLog = new LogWrapper(ElevatorSys.class.getName());
    private static final PeriodicLogger mPLog = new PeriodicLogger(mLog, 50);

    public static ElevatorSys mInstance;

    public static enum Direction {
        Up, Down
    }

    private static final double GEAR_DIA_INCHES = 1.5;

    // a motor output of BASE_POWER holds the motor in place when not disturbed
    public static final double BASE_PERCENT_OUT = RobotConfig.ELV_BASE_PERCENT_OUT;

    public static final double MANUAL_POWER_UP = BASE_PERCENT_OUT + 0.6;

    public static final double MANUAL_POWER_DOWN = -0.4;

    // MEASURE the ticks per inch on physical mechanism
    public static final int TICKS_PER_INCH = RobotConfig.ELV_TICKS_PER_INCH; // MEASURED
    public static final double INCHES_PER_REVOLUTION = 4096 / TICKS_PER_INCH;

    public static final double ELEVATOR_FLOOR_INCHES = 14.5; // inches from ground when elevator at zero
                                                              // measured from wrist axel to floor but does not account
                                                              // for ball center
    

    private ElevatorSys.ElvTarget m_targ;
    private TTPIDController m_movePID;
    private TTPIDController m_holdPID;
    private PIDSourceTalonPW m_PidSourceTalonPW;
    private PIDOutTalon m_PidOutTalon;
    private PIDOutTalon m_HoldPidOutTalon;

    private boolean m_usingHoldPID;

    /**
     * How many sensor units per rotation.
     * 
     * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
     */
    private static final int kCTREUnitsPerRotation = 4096; // 4096;

    private static final int kUnitsPerRotation = kCTREUnitsPerRotation;

    // inches of elevator travel per complete rotation of encoder gear is 1 inch
    // diameter
    private static final double kDistancePerRotation = 1.75 * Math.PI;

    private static final int kUnitsPerInch = (int) Math.round(kUnitsPerRotation / kDistancePerRotation);

    // talon setup
    // -------------------------------------------------------------------------------

    private WPI_TalonSRX mTalon;
    private WPI_TalonSRX mTalon_Slave0;

    private static final boolean TALON_INVERT = RobotConfig.ELV_INVERT;
    private static final boolean TALON_SENSOR_PHASE = RobotConfig.ELV_SENSOR_PHASE;

    private static final int TALON_FORWARD_LIMIT = -1;

    private static final int TALON_REVERSE_LIMIT = -1;

    /*
     * set the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    private static final int TALON_ALLOWED_CLOSELOOP_ERROR = 0;

    public static final int kTimeoutMs = 10;

    // Motor deadband, set to 1%.
    public static final double kNeutralDeadband = 0.01;
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
     * or 3. Only the first two (0,1) are visible in web-based configuration.
     */
    public static final int kPIDSlot_Move = 1;
    public static final int kPIDSlot_Hold = 0;
    public static final int kPIDSlot_2 = 2;
    public static final int kPIDSlot_3 = 3;

    public static final int kPIDLoopIdx = 0;

    // Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
    // public static final int kPIDLoopIdx = 0;

    // paramter to the configXXX() methods. Set to non-zero to have talon wait to
    // check and report error
    // public static final int kTimeoutMs = 10;

    /**
     * Specify the target position we want to reach. Might be replaced by an enum or
     * some other way of specifying desired state
     */
    private double mTarget;

    // ---------------Rocket Hatch---------

    private static final double ROCKET_HATCH_LO_INCHES = ((12 + 7) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_HATCH_LO = (int) (ROCKET_HATCH_LO_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_HATCH_MID_INCHES = (ROCKET_HATCH_LO_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_MID = (int) (ROCKET_HATCH_MID_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_HATCH_HI_INCHES = (ROCKET_HATCH_MID_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_HI = (int) (ROCKET_HATCH_HI_INCHES * ElevatorSys.TICKS_PER_INCH);

    // ---------------Rocket Cargo-------------

    private static final double ROCKET_CARGO_LO_INCHES = ((24 + 3.5) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_CARGO_LO = (int) (ROCKET_CARGO_LO_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_CARGO_MID_INCHES = (ROCKET_CARGO_LO_INCHES + 24 + 4);
    private static final int ROCKET_CARGO_MID = (int) (ROCKET_CARGO_MID_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_CARGO_HI_INCHES = (ROCKET_CARGO_MID_INCHES + 24 + 4);
    private static final int ROCKET_CARGO_HI = (int) (ROCKET_CARGO_HI_INCHES * ElevatorSys.TICKS_PER_INCH);

    // -----------------Cargoship Hatch--------

    private static final double CARGOSHIP_HATCH_INCHES = ((12 + 7) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int CARGOSHIP_HATCH = (int) (CARGOSHIP_HATCH_INCHES * ElevatorSys.TICKS_PER_INCH);

    // ----------------CARGOSHIP CARGO---------

    private static final double CARGOSHIP_CARGO_INCHES = ((24 + 7.5 + 6.5 + 2) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    // extra 2 inches for safety^^^
    private static final int CARGOSHIP_CARGO = (int) (CARGOSHIP_CARGO_INCHES * ElevatorSys.TICKS_PER_INCH);

    public enum ElvTarget {
        RocketHatchHi(ROCKET_HATCH_HI), RocketHatchMid(ROCKET_HATCH_MID), RocketHatchLo(ROCKET_HATCH_LO),
        RocketCargoHi(ROCKET_CARGO_HI), RocketCargoMid(ROCKET_CARGO_MID), RocketCargoLo(ROCKET_CARGO_LO),
        CargoshipHatch(CARGOSHIP_HATCH), CargoshipCargo(CARGOSHIP_CARGO), HatchPickUp(CARGOSHIP_HATCH);

        private int mTicks;

        ElvTarget(int ticks) {
            mTicks = ticks;
        }

        public int getTicks() {
            return mTicks;
        }
    }

    /**
     * Log the sensor position at power up - use this as the base reference for
     * positioning.
     */
    private int mBasePosn;

    // specify the boundaries beyond which not allowed to have power
    public static int MAX_TRAVEL = 17000;
    public static int MIN_TRAVEL = (int)(TICKS_PER_INCH * 2);

    private DigitalInput m_BottomLimit;
    private Counter m_BottomLimitCtr;
    private Notifier m_BotLimitWatcher;
    private boolean m_botLimitSwitchActive = false;

    private enum ELV_STATE {
        IDLE, MANUALUP, MANUALDOWN, HOLD, MOVEUPSLOW, MOVETO, PIDMOVE, PIDHOLD, DISABLED, WD_NOUP, WD_NODOWN
    }

    private ELV_STATE m_curState;

    public static ElevatorSys getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSys();
        }
        return mInstance;
    }

    @Override
    public void initDefaultCommand() {
        // setDefaultCommand(new ElvMoveUpSlow());
    }

    public ElevatorSys() {
        mLog.info("ElevatorSys ctor  ----------------------------------------------");
        try {
            mTalon = new WPI_TalonSRX(RobotConfig.ELEVATOR_MASTER);
            mTalon.configFactoryDefault();
            mTalon.setSafetyEnabled(false);
            mTalon.setName(String.format("%d: Elevator", RobotConfig.ELEVATOR_MASTER));
            // in case we are in magic motion or position hold mode
            mTalon.set(ControlMode.PercentOutput, 0);

            mTalon.setSensorPhase(TALON_SENSOR_PHASE);
            mTalon.setInverted(TALON_INVERT);

            if (RobotConfig.IS_ROBO_2019) {
                mTalon_Slave0 = new WPI_TalonSRX(RobotConfig.ELEVATOR_SLAVE0);
                mTalon_Slave0.configFactoryDefault();
                mTalon_Slave0.follow(mTalon, FollowerType.PercentOutput);
                mTalon_Slave0.setInverted(InvertType.FollowMaster); // follow tested 2-19
            }

            mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
            mTalon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

            mTalon.configOpenloopRamp(0.2, kTimeoutMs);
            mTalon.setNeutralMode(NeutralMode.Brake);

            // mTalon.configForwardSoftLimitThreshold(TALON_FORWARD_LIMIT, kTimeoutMs);
            // mTalon.configForwardSoftLimitEnable(false, kTimeoutMs);
            // mTalon.configReverseSoftLimitThreshold(TALON_REVERSE_LIMIT, kTimeoutMs);
            // mTalon.configReverseSoftLimitEnable(false, kTimeoutMs);

            // see setup for motion magic in s/w manual 12.6
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);

            // set up current limits
            mTalon.configPeakCurrentLimit(50, kTimeoutMs);
            mTalon.configPeakCurrentDuration(200, kTimeoutMs);
            mTalon.configContinuousCurrentLimit(1, kTimeoutMs);
            mTalon.enableCurrentLimit(true);

            // nominal values are used in closed loop when in deadband
            mTalon.configNominalOutputForward(0, kTimeoutMs);
            mTalon.configNominalOutputReverse(0, kTimeoutMs);
            mTalon.configPeakOutputForward(1.0, kTimeoutMs);
            mTalon.configPeakOutputReverse(-1.0, kTimeoutMs);

            // PID settings

            // ramping can interfere with the PID:
            // https://www.chiefdelphi.com/forums/showthread.php?p=1748993#post1748993
            // mTalon.configClosedloopRamp(0.1, kTimeoutMs);

            // bot to top 29000 in 1 sec = 2900 ticks per 100 ms
            mTalon.configMotionCruiseVelocity(3500, kTimeoutMs); // 2900
            mTalon.configMotionAcceleration(2500, kTimeoutMs); // 2000
            mTalon.configAllowableClosedloopError(kPIDSlot_Move, TALON_ALLOWED_CLOSELOOP_ERROR, kTimeoutMs);

            m_BottomLimit = new DigitalInput(2);
            m_BottomLimit.setName("ElevatorSys", "BotLimit");
            m_BottomLimitCtr = new Counter(m_BottomLimit);
            m_BottomLimitCtr.reset();
            m_BotLimitWatcher = new Notifier(this::botLimitWatcher);
            m_BotLimitWatcher.startPeriodic(0.01); // check the limit switch every 10 milliSec

            m_PidSourceTalonPW = new PIDSourceTalonPW(mTalon, 0);

            setSensorStartPosn();

            m_curState = ELV_STATE.IDLE;

            // set the watch dog going
            mWatchDogTimer = new Timer("ElvSys watchdog");
            // // wait for 1 second before starting, then check every 50 milliseconds
            mWatchDogTimer.schedule(mWatchDog, 1000, 50);

            mLog.info("ElevatorSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "ElevatorSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }


    /**
     * Safely transition from one stae to another
     * @param newState
     */
    private void setState(ELV_STATE newState) {

        mLog.debug("ES.setState:  cur: %s   new: %s", m_curState.toString(), newState.toString());
        if (m_curState == ELV_STATE.IDLE 
                || m_curState == ELV_STATE.MANUALUP
                || m_curState == ELV_STATE.MANUALDOWN
                || m_curState == ELV_STATE.HOLD
                || m_curState == ELV_STATE.MOVEUPSLOW
                || m_curState == ELV_STATE.MOVETO
                || m_curState == ELV_STATE.DISABLED) {
            // always safe to move to new state
            m_curState = newState;
            return;
        }
        if (m_curState == ELV_STATE.PIDMOVE) {

        }
        else if (m_curState == ELV_STATE.PIDHOLD) {

        }
    }


    // ---------- WatchDog  -----------------------------

    public void killWatchDog() {
        mLog.info("Killing Watchdog");
        MAX_TRAVEL += 20000;
        MIN_TRAVEL -= 20000;
    }

    public void reviveWatchDog() {
        mTalon.setSelectedSensorPosition(0);
        MAX_TRAVEL -= 20000;
        MIN_TRAVEL += 20000;
        mLog.debug("Reviving Watch Dog");
    }

    /**
     * Called every N milliSeconds by Notifier to check the state of the limit
     * switch Limit switch will be set when elevator at bottom - we do not move
     * through and keep going, so no need to watch a counter
     */
    private void botLimitWatcher() {
        m_botLimitSwitchActive = m_BottomLimit.get();
    }

    // -------set up watch on talon position, disable if out of bounds ---------------

    private Timer mWatchDogTimer = new Timer();

    private boolean m_DontMoveDown = false;
    private boolean m_DontMoveUp = false;

    private TimerTask mWatchDog = new TimerTask() {
        public void run() {
            int curPosn = mTalon.getSelectedSensorPosition();
            double curOutput = mTalon.getMotorOutputPercent();
            if (curPosn > (MAX_TRAVEL + 100)) {
                // past the max boundry and going forward
                m_DontMoveDown = false;
                m_DontMoveUp = true;
                mTalon.set(ControlMode.PercentOutput, 0);
                setState(ELV_STATE.WD_NOUP);
                mLog.severe("ElvSys: talon exceeded top top boundry \n Killing Elv Current");
            }else if (curPosn > (MAX_TRAVEL) && curOutput > 0) {
                // past the max boundry and going forward
                m_DontMoveDown = false;
                m_DontMoveUp = true;
                mTalon.set(ControlMode.PercentOutput, BASE_PERCENT_OUT);
                setState(ELV_STATE.WD_NOUP);
                mLog.severe("ElvSys: talon exceeded top boundry");
            // } else if (curPosn < MIN_TRAVEL && curOutput < 0) {
            //     // past the max boundry and going forward
            //     m_DontMoveDown = true;
            //     m_DontMoveUp = false;
            //     mTalon.set(ControlMode.PercentOutput, BASE_PERCENT_OUT);
            //     setState(ELV_STATE.WD_NODOWN);
            //     mLog.severe("ElvSys: talon exceeded bottom boundry");
            } else {
                m_DontMoveDown = false;
                m_DontMoveUp = false;
            }
        }
    };


    /**
     * Disable the elevator system - make sure all talons and PID loops are not
     * driving anything
     */
    public void disable() {
        mLog.debug("ElvSys DISABLED  <<<<<<<<<<<<<<<<<<<<");
        setState(ELV_STATE.DISABLED);
        if (m_movePID != null) {
            m_movePID.disable();
        }
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        if (m_BotLimitWatcher != null) {
            m_BotLimitWatcher.stop();
        }
        mTalon.set(ControlMode.PercentOutput, 0);
    }

    //-----------------------------------------------------

    // grab the 360 degree position of the MagEncoder's absolute position, and set
    // the relative sensor to match.
    // should only be called on robot.init
    public void setSensorStartPosn() {

        // mTalon.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
        // // mBasePosn = mTalon.getSensorCollection().getPulseWidthPosition();
        // int absolutePosition = mBasePosn;
        // /* mask out overflows, keep bottom 12 bits */
        // absolutePosition &= 0xFFF;
        // if (TALON_SENSOR_PHASE)
        //     absolutePosition *= -1;
        // if (TALON_INVERT)
        //     absolutePosition *= -1;
        // /* set the quadrature (relative) sensor to match absolute */
        // mTalon.setSelectedSensorPosition(absolutePosition, 0, kTimeoutMs);
        mTalon.setSelectedSensorPosition(0);
        mBasePosn = mTalon.getSelectedSensorPosition(0);
        mLog.debug(printPosn("setStart"));
    }


    public String printPosn(String caller) {
        int sensPosn = mTalon.getSelectedSensorPosition(0);
        String sensPosnSign = "(+)";
        int absSensPosn = Math.abs(sensPosn);

        if (sensPosn < 0) {
            // absSensPosn is negative if moving down
            sensPosnSign = "(-)";
        }
        int curSensPosn = mTalon.getSelectedSensorPosition(0);
        double vel = mTalon.getSensorCollection().getQuadratureVelocity();
        double mout = mTalon.getMotorOutputPercent();
        double voltOut = mTalon.getMotorOutputVoltage();
        double curOut = mTalon.getOutputCurrent();

        return String.format(
                "ES.%s  AtBase: %b LimCnt: %d   base: %d  curPosn: %d  vel: %.3f  pcOut: %.3f  volts: %.3f  cur: %.3f",
                caller, m_botLimitSwitchActive, m_BottomLimitCtr.get(), mBasePosn, curSensPosn, vel, mout, voltOut,
                curOut);
    }


    // MovSlowUpCmd --------------------------------------------------------

    // move up very slowly until we have moved 2 inches. Idea is to find minimum power
    // need to move the elevator up, because it is very negatively weighted

    private int mStartPosn = 0;

    private double mPercentOut;

    public void initMovSlowUp() {
        setState(ELV_STATE.MOVEUPSLOW);
        m_curState = ELV_STATE.MANUALUP;
        mStartPosn = mTalon.getSensorCollection().getPulseWidthPosition();
        mPercentOut = 0.0;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mLog.debug(printPosn("initMovSlowUp"));
    }

    public void execMovSlowUp() {
        mPercentOut += 0.001;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execMovSlowUp"));
    }

    public boolean isCompleteMovSlowUp() {
        int curPosn = mTalon.getSensorCollection().getPulseWidthPosition();
        boolean isFin = (curPosn - mStartPosn) >= TICKS_PER_INCH * 2;
        if (isFin) {
            mLog.debug(printPosn("isComp") + "\n------------------------------------------------------");
            mLog.debug("Holding at output %.3f", BASE_PERCENT_OUT);
            mPercentOut = BASE_PERCENT_OUT;
            mTalon.set(ControlMode.PercentOutput, mPercentOut);
        }
        return isFin;
    }


    // ---- basic hold called from ElvHoldCmd-------------------------------------

    /**
     * Hold the elevator at current position using just set power
     */
    public void holdPosnPower() {
        setState(ELV_STATE.HOLD);
        mLog.debug(printPosn("holdPosnPower") + "\n------------------------------------------------------");
        mLog.debug("Holding at output %.3f", BASE_PERCENT_OUT);
        mPercentOut = BASE_PERCENT_OUT;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
    }


    // -----------Move To without PID -----------------------------------------

    private final double AUTO_SPEED = 0.3;
    private boolean ElvMoveUpPolarity;

    public void initMoveToWithoutPID(Objective.ElvTarget target) {
        setState(ELV_STATE.MOVETO);
        int curPosition = mTalon.getSelectedSensorPosition();
        int targetPosition = target.getTicks();
        if ((targetPosition - curPosition) > 0) {
            ElvMoveUpPolarity = true;
        } else {
            ElvMoveUpPolarity = false;
        }
    }

    public void execMoveToWithoutPID(Objective.ElvTarget target) {
        int curPosition = mTalon.getSelectedSensorPosition();
        int targetPosition = target.getTicks();
        if (ElvMoveUpPolarity && (curPosition < targetPosition)) {
            mTalon.set(ControlMode.PercentOutput, AUTO_SPEED);
        } else if (!ElvMoveUpPolarity && (targetPosition < curPosition)) {
            mTalon.set(ControlMode.PercentOutput, -AUTO_SPEED);
        } else {
            mTalon.set(ControlMode.PercentOutput, BASE_PERCENT_OUT);
        }
    }

    public boolean isFinishedMoving(Objective.ElvTarget target) {
        int curPosition = mTalon.getSelectedSensorPosition();
        int targetPosition = target.getTicks();
        if (ElvMoveUpPolarity && (curPosition > targetPosition)) {
            mTalon.set(ControlMode.PercentOutput, BASE_PERCENT_OUT);
            return true;
        } else if (!ElvMoveUpPolarity && (targetPosition > curPosition)) {
            mTalon.set(ControlMode.PercentOutput, BASE_PERCENT_OUT);
            return true;
        } else {
            return false;
        }
    }

    // ------------------ Move Up ---------------------------------

    /**
     * Move up at 0.3 power more than hold
     */
    public void initMoveUp() {
        setState(ELV_STATE.MANUALUP);
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        if (m_movePID != null) {
            m_movePID.disable();
        }
        if (m_DontMoveUp) {
            return;
        }
        mStartPosn = mTalon.getSelectedSensorPosition();
        mPercentOut = MANUAL_POWER_UP;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mLog.debug("********************");
        mLog.debug(printPosn("initMoveUp"));
    }

    public void execMoveUp() {
        execMoveUp(MANUAL_POWER_UP);
    }
    
    public void execMoveUp(double pcOut) {
        if (m_DontMoveUp) {
            return;
        }
        // mPercentOut = MANUAL_POWER_UP;
        mTalon.set(ControlMode.PercentOutput, pcOut);
        mPLog.debug(printPosn("execMoveUp"));
    }

    // ---------- Move Down  -------------------------------

    /**
     * Move down at -0.1 power
     */
    public void initMoveDown() {
        setState(ELV_STATE.MANUALDOWN);
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        if (m_movePID != null) {
            m_movePID.disable();
        }
        if (m_DontMoveDown) {
            return;
        }
        mStartPosn = mTalon.getSelectedSensorPosition();
        mPercentOut = MANUAL_POWER_DOWN;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mLog.debug(printPosn("initMoveDown"));
    }

    public void execMoveDown() {
        execMoveDown(MANUAL_POWER_DOWN);
    }
    
    public void execMoveDown(double pcOut) {
        if (m_DontMoveDown) {
            return;
        }
        mTalon.set(ControlMode.PercentOutput, pcOut);
        mPLog.debug(printPosn("execMoveDown"));
    }


    // ------ hold posn PID using the TritonTech PID -------------------

    /**
     * Sensor is on output of gearing (not on motor) Set the tolerance to +- 0.5
     * inches
     */
    public void initHoldPosnPID() {
        if (m_holdPID == null) {
            mLog.debug(printPosn("initHoldPosnPID:"));
            m_HoldPidOutTalon = new PIDOutTalon(mTalon, 0.1, -0.8, 0.8);
            double kP = 0.01 / 500; // want 20% power when hit tolerance band of 500 units (was 0.001)
            double kI = 0.0; //0.000001;
            double kD = 0.0;
            double kF = 0.0;
            double periodInSecs = 0.05; // for hold, check every 50 mS is fine
            m_holdPID = new TTPIDController("elvHold", kP, kI, kD, kF, m_PidSourceTalonPW, m_HoldPidOutTalon, periodInSecs, null);
            m_holdPID.setAbsoluteTolerance(0.5 * TICKS_PER_INCH); // allow +- 200 units (0.4 inches) on error
            m_holdPID.setRamp(0.0, 0.0); // no ramping on a hold
            m_holdPID.setDebugEnabled(true, 10);
        } else {
            m_holdPID.reset();
        }
    }

    /**
     * Hold at the current position
     */
    public void enableHoldPosnPID() {
        int curPosn = mTalon.getSelectedSensorPosition();
        enableHoldPosnPID(curPosn);
    }

    /**
     * Do a PID hold at the specified sensor position
     */
    public void enableHoldPosnPID(int targetPosn) {
        setState(ELV_STATE.PIDHOLD);
        if (m_holdPID == null) {
            initHoldPosnPID();
        }
        // if(mClimbPidController != null){
        //     mClimbPidController.disable();
        // }
        m_holdPID.reset();
        mLog.debug("ES.enableHoldPosnPID: target: %d    ---------------------", targetPosn);
        mLog.debug(printPosn("enableHoldPosnPID"));
        m_holdPID.setSetpoint(targetPosn);
        m_holdPID.setDebugEnabled(true, 10);
        m_holdPID.enable();
    }

    /**
     * Disable the hold PID. This will send 0 to the PID out, which writes to the
     * talon
     */
    public void disableHoldPosnPID() {
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        // if(mClimbPidController != null){
        //     mClimbPidController.disable();
        // }
        mTalon.set(ControlMode.PercentOutput, 0.0);
        mLog.debug(printPosn("disableHoldPosnPID"));
    }

    // move to target using PID ---------------------------------------------

    private boolean m_haveToStop;
    /**
     * Target assumes that the elevator base position is zero Need to adjust for the
     * actual sensor start position
     * 
     * @param targ
     */
    public void initPIDMoveToTarget(ElevatorSys.ElvTarget targ) {
        setState(ELV_STATE.PIDMOVE);
        m_targ = targ;
        initHoldPosnPID();

        // if(mClimbPidController != null){
        //     mClimbPidController.disable();
        // }
        if (m_usingHoldPID) {
            m_haveToStop = true;
            m_holdPID.disable();
        }
        if (m_movePID == null) {
            m_PidOutTalon = new PIDOutTalon(mTalon, 0.3, -0.8, 0.8);
            double kP = 0.05 / 500; // want 20% power when hit tolerance band of 500 units (was 0.001)
            double kI = 0.0;
            double kD = 0.0;
            double kF = 0.0;
            double periodInSecs = 0.05; // run PID calc every 50 mS
            m_movePID = new TTPIDController("elvM2Targ", kP, kI, kD, kF, m_PidSourceTalonPW, m_PidOutTalon,
                    periodInSecs, this);
            m_movePID.setAbsoluteTolerance(2 * TICKS_PER_INCH); // allow +- one inch - then hand over to posn hold to lock in
            m_movePID.setDebugEnabled(true, 10);
        } else {
            m_movePID.reset();
        }

        int curPosn = mTalon.getSelectedSensorPosition(0);
        int calcTarg = targ.getTicks();
        mLog.debug("initMoveToTarget: curPos: %d    targ: %s(%d)  calcTarg: %d  ---------------------", curPosn,
                m_targ.toString(), m_targ.getTicks(), calcTarg);
        mLog.debug(printPosn("initMoveToTarget"));
        m_usingHoldPID = false;
        m_movePID.setSetpoint(calcTarg);
        //m_movePID.setRamp(2 * TICKS_PER_INCH, 2 * TICKS_PER_INCH); // set ramps to 3 inches
        m_movePID.setBasePower(0.3, 0.05);
        m_movePID.enable();
    }

    /**
     * Dont need to actually do anything here, because the PID if writing to the
     * Talon What we want to do is wait until the PID is close, then use the holdPID
     * to lock in
     */
    // public void execMoveToTarget() {
    //     mPLog.debug("ES.execMoveToTarg:  onTarg: %b  curPosn: %d  pidOut: %.3f  holdPidOut: %.3f", m_movePID.onTarget(),
    //             mTalon.getSelectedSensorPosition(), m_PidOutTalon.getVal(), m_HoldPidOutTalon.getVal());
    //     if (m_movePID.onTarget() && !m_usingHoldPID) {
    //         mLog.debug("ES.execMoveToTarget: on target  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    //         mLog.debug(printPosn("ES.execMoveToTarget:"));
    //         enableHoldPosnPID(m_targ.getTicks());
    //         m_movePID.disable();
    //         m_usingHoldPID = true;
    //     }
    // }


    /**
     * This is passed to the movePID controller, which will call it when it hits target
     * The method returns true if we want to PID controller to disable itself
     */
    public boolean PID_ExecOnTarget() {
        mLog.debug("ES.pidExecOnTarget: on target  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        mLog.debug(printPosn("ES.pidExecOnTarget:"));
        enableHoldPosnPID(m_targ.getTicks());
        m_usingHoldPID = true;
        return true;    // force movePID to disable
    }


    /**
     * Only return true once we have moved to using holdPID and it is on target
     */
    public boolean isMoveToTargetComplete() {
        if (m_usingHoldPID) {
            if (m_holdPID.onTarget()) {
                mPLog.debug("ES.isMoveToTargetComplete:   curPosn: %d  pidOut: %.3f  holdPidOut: %.3f",
                        mTalon.getSelectedSensorPosition(), m_PidOutTalon.getVal(), m_HoldPidOutTalon.getVal());

            }
            return m_holdPID.onTarget();
        }
        if (m_haveToStop) {
            // nasty hack to ensure that previous command that did not complete with on target gets stopped
            m_haveToStop = false;
            return true;
        }
        return false;
    }

    
    public void disableMoveToPID() {
        if (m_movePID != null) {
            m_movePID.disable();
        }
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        // if(mClimbPidController != null){
        //     mClimbPidController.disable();
        // }
        mTalon.set(ControlMode.PercentOutput, 0.0);
        mLog.debug(printPosn("disableMoveToPID"));
    }


    public WPI_TalonSRX getTalon() {
        return mTalon;
    }

}
