package team6072.robo2019.subsystems;

import java.util.TimerTask;
import java.util.Timer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import team6072.robo2019.pid.TTPIDController;

/**
 * Add your docs here.
 */
public class WristSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(WristSys.class.getName());

    private static WristSys mInstance;

    public static enum Direction {
        Up, Down
    }

    // a motor output of BASE_POWER holds the motor in place when not disturbed
    private static final double BASE_PERCENT_OUT = RobotConfig.WRIST_BASE_PERCENT_OUT;

    // consider 0 degreess as fully folded back onto arm
    // 180 degrees folded out so presenting flat to target

    private static final int BASE = 0;

    public static final double STARTING_ANGLE = 35.0;
    private static final double TILT_TO_TOP_CARGO_ROCKET = 165; // ESTIMATE
    private static final double FLAT_TO_TARGET = 180; // ESTIMATE
    private static final double TILT_TO_GROUND = 190; // Estimate

    // MEASURE the ticks per degree on physical mechanism
    private static final int TICKS_PER_DEG = RobotConfig.WRIST_TICKS_PER_DEG; // MEASURED

    // specify the boundaries beyond which not allowed to have power
    private static final double RETRACT_STOP_ANGLE = 37.0;
    private static final int MAX_TRAVEL = (int) ((RETRACT_STOP_ANGLE - STARTING_ANGLE) * TICKS_PER_DEG);
    private static final double EXTEND_STOP_ANGLE = 250.0;
    private static final int MIN_TRAVEL = (int) ((EXTEND_STOP_ANGLE - STARTING_ANGLE) * TICKS_PER_DEG);

    /*
     * // --------------------------------------Rocket //
     * Hatch----------------------------------------------
     * 
     * private static final int ROCKET_HATCH_LO_DEGS = FLAT_TO_TARGET; private
     * static final int ROCKET_HATCH_LO = (int) (ROCKET_HATCH_LO_DEGS *
     * TICKS_PER_DEG);
     * 
     * private static final int ROCKET_HATCH_MID_DEGS = FLAT_TO_TARGET; private
     * static final int ROCKET_HATCH_MID = (int) (ROCKET_HATCH_MID_DEGS *
     * TICKS_PER_DEG);
     * 
     * private static final int ROCKET_HATCH_HI_DEGS = FLAT_TO_TARGET; private
     * static final int ROCKET_HATCH_HI = (int) (ROCKET_HATCH_HI_DEGS *
     * TICKS_PER_DEG);
     * 
     * // -------------------------------------Rocket //
     * Cargo----------------------------------------------
     * 
     * private static final int ROCKET_CARGO_LO_DEGS = FLAT_TO_TARGET; private
     * static final int ROCKET_CARGO_LO = (int) (ROCKET_CARGO_LO_DEGS *
     * TICKS_PER_DEG);
     * 
     * private static final int ROCKET_CARGO_MID_DEGS = FLAT_TO_TARGET; private
     * static final int ROCKET_CARGO_MID = (int) (ROCKET_CARGO_MID_DEGS *
     * TICKS_PER_DEG);
     * 
     * private static final int ROCKET_CARGO_HI_DEGS = TOP_CARGO_ROCKET; private
     * static final int ROCKET_CARGO_HI = (int) (ROCKET_CARGO_HI_DEGS *
     * TICKS_PER_DEG);
     * 
     * // --------------------------------------Cargoship //
     * Hatch----------------------------------------
     * 
     * private static final int CARGOSHIP_HATCH_DEGS = FLAT_TO_TARGET; private
     * static final int CARGOSHIP_HATCH = (int) (CARGOSHIP_HATCH_DEGS *
     * TICKS_PER_DEG);
     * 
     * // --------------------------------------CARGOSHIP //
     * CARGO----------------------------------------
     * 
     * private static final int CARGOSHIP_CARGO_DEGS = FLAT_TO_TARGET;
     * 
     * private static final int CARGOSHIP_CARGO = (int) (CARGOSHIP_CARGO_DEGS *
     * TICKS_PER_DEG);
     * 
     */ public enum WristTarget {
        RetractedStartPosition(STARTING_ANGLE), RetrieveBallPosition(TILT_TO_GROUND),
        FlatDeployPosition(FLAT_TO_TARGET), TiltedDeployCargoHi(TILT_TO_TOP_CARGO_ROCKET);
        /*
         * RocketHatchHi(ROCKET_HATCH_HI), RocketHatchMid(ROCKET_HATCH_MID),
         * RocketHatchLo(ROCKET_HATCH_LO), RocketCargoHi(ROCKET_CARGO_HI),
         * RocketCargoMid(ROCKET_CARGO_MID), RocketCargoLo(ROCKET_CARGO_LO),
         * CargoshipHatch(CARGOSHIP_HATCH), CargoshipCargo(CARGOSHIP_CARGO);
         */

        private double mAngle;

        WristTarget(double angle) {
            mAngle = angle;
        }

        public int getTicks() {
            mAngle -= STARTING_ANGLE;
            return (int) (mAngle * TICKS_PER_DEG);
        }
    }

    private WristTarget m_targ;
    private TTPIDController m_movePID;
    private TTPIDController m_holdPID;
    private PIDSourceTalonPW m_PidSourceTalonPW;
    private PIDOutTalon m_PidOutTalon;

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
    // private WPI_TalonSRX mTalon_Slave0; // ctr + f slave to find them

    private static final boolean TALON_INVERT = RobotConfig.WRIST_INVERT;
    private static final boolean TALON_SENSOR_PHASE = RobotConfig.WRIST_SENSOR_PHASE;

    private static final int TALON_FORWARD_LIMIT = 1;

    private static final int TALON_REVERSE_LIMIT = -1;

    public static final int kTimeoutMs = 10;

    public static final int kPIDLoopIdx = 0;

    // Motor deadband, set to 1%.
    public static final double kNeutralDeadband = 0.01;

    /**
     * Log the sensor position at power up - use this as the base reference for
     * positioning.
     */
    private int mBasePosn;

    public static WristSys getInstance() {
        if (mInstance == null) {
            mInstance = new WristSys();
        }
        return mInstance;
    }

    @Override
    public void initDefaultCommand() {
        // setDefaultCommand(new WristMoveUpSlow());
    }

    public WristSys() {
        mLog.info("WristSys ctor  ----------------------------------------------");
        try {
            mTalon = new WPI_TalonSRX(RobotConfig.WRIST_MASTER);
            mTalon.configFactoryDefault();
            mTalon.setName(String.format("Wrist: %d", RobotConfig.WRIST_MASTER));
            // in case we are in magic motion or position hold mode
            mTalon.set(ControlMode.PercentOutput, 0);

            mTalon.setSensorPhase(TALON_SENSOR_PHASE);
            mTalon.setInverted(TALON_INVERT);
            mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
            mTalon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

            if (RobotConfig.IS_ROBO_2019) {
                // mTalon_Slave0 = new WPI_TalonSRX(RobotConfig.WRIST_SLAVE0);
                // mTalon_Slave0.follow(mTalon, FollowerType.PercentOutput);
                // mTalon_Slave0.setInverted(InvertType.FollowMaster);
            }

            // mTalon.configForwardSoftLimitThreshold(TALON_FORWARD_LIMIT, kTimeoutMs);
            // mTalon.configForwardSoftLimitEnable(false, kTimeoutMs);
            // mTalon.configReverseSoftLimitThreshold(TALON_REVERSE_LIMIT, kTimeoutMs);
            // mTalon.configReverseSoftLimitEnable(false, kTimeoutMs);

            mTalon.configOpenloopRamp(0.1, kTimeoutMs);
            mTalon.setNeutralMode(NeutralMode.Brake);

            // set up current limits
            mTalon.configPeakCurrentLimit(40, kTimeoutMs);
            mTalon.configPeakCurrentDuration(200, kTimeoutMs);
            mTalon.configContinuousCurrentLimit(10, kTimeoutMs);
            mTalon.enableCurrentLimit(true);

            m_PidSourceTalonPW = new PIDSourceTalonPW(mTalon, 0);

            setSensorStartPosn();

            // // set the watch dog going
            // mWatchDogTimer = new Timer("WristSys watchdog");
            // // wait for 1 second before starting, then check every 50 milliseconds
            // mWatchDogTimer.schedule(mWatchDog, 1000, 50);

            mLog.info("WristSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "WristSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }

    /**
     * Disable the elevator system - make sure all talongs and PID loops are not
     * driving anything
     */
    public void disable() {
        mLog.debug("WristSys DISABLED  <<<<<<<<<<<<<<<<<<<<");
        if (m_movePID != null) {
            m_movePID.disable();
        }
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        mTalon.set(ControlMode.PercentOutput, 0);
    }

    public void feedTalons() {
        mTalon.feed();
        if (RobotConfig.IS_ROBO_2019) {
            // mTalon_Slave0.feed();
        }
    }

    // ------------ set up watch on talon position and disable if out of bounds
    // -----------------------

    private Timer mWatchDogTimer = new Timer();

    private boolean mDontExtend = false;
    private boolean mDontRetract = false;

    private TimerTask mWatchDog = new TimerTask() {
        public void run() {
            int curPosn = getWristPosition();
            double curOutput = mTalon.getMotorOutputPercent();
            if (curPosn > MAX_TRAVEL && curOutput > 0) {
                // past the max boundry and going forward
                mDontExtend = true;
                mDontRetract = false;
                mTalon.set(ControlMode.PercentOutput, 0);
                mLog.severe("WristSys: talon exceeded forward boundry");
            } else if (curPosn < MIN_TRAVEL && curOutput < 0) {
                mDontExtend = false;
                mDontRetract = true;
                // past the max boundry and going forward
                mTalon.set(ControlMode.PercentOutput, 0);
                mLog.severe("WristSys: talon exceeded backward boundry");
            } else {
                mDontExtend = false;
                mDontRetract = false;
            }
        }
    };

    // --------------------------------------------------------------------------------------------------

    // grab the 360 degree position of the MagEncoder's absolute position, and set
    // the relative sensor to match.
    // should only be called on robot.init
    public void setSensorStartPosn() {
        mTalon.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
        // mBasePosn = getWristPosition();
        int absolutePosition = mBasePosn;
        /* mask out overflows, keep bottom 12 bits */
        absolutePosition &= 0xFFF;
        if (TALON_SENSOR_PHASE)
            absolutePosition *= -1;
        if (TALON_INVERT)
            absolutePosition *= -1;
        /* set the quadrature (relative) sensor to match absolute */
        mTalon.setSelectedSensorPosition(absolutePosition, 0, kTimeoutMs);
        mBasePosn = mTalon.getSelectedSensorPosition(0);
        mLog.debug(printPosn("setStart"));
    }

    public int getWristPosition() {
        return mTalon.getSelectedSensorPosition();
    }

    private double mLastSensPosn;
    private double mLastQuadPosn;

    public String printPosn(String caller) {
        int sensPosn = mTalon.getSelectedSensorPosition(0);
        String sensPosnSign = "(+)";
        int absSensPosn = Math.abs(sensPosn);

        if (sensPosn < 0) {
            // absSensPosn is negative if moving down
            sensPosnSign = "(-)";
        }
        int quadPosn = getWristPosition();
        int pwPosn = getWristPosition();
        int selSensPosn = mTalon.getSelectedSensorPosition(0);
        int pwDelta = pwPosn - mBasePosn;
        double pwVel = mTalon.getSensorCollection().getPulseWidthVelocity();
        int relDelta = absSensPosn - mBasePosn;
        int quadDelta = quadPosn - mBasePosn;
        double vel = mTalon.getSensorCollection().getQuadratureVelocity();
        double mout = mTalon.getMotorOutputPercent();
        double voltOut = mTalon.getMotorOutputVoltage();
        double curOut = mTalon.getOutputCurrent();
        mLastSensPosn = absSensPosn;
        mLastQuadPosn = quadPosn;
        return String.format("WS.%s  base: %d  selPosn: %d  vel: %.3f  pcOut: %.3f  volts: %.3f  cur: %.3f", caller,
                mBasePosn, selSensPosn, vel, mout, voltOut, curOut);
    }

    // MovSlowUpCmd --------------------------------------------------------

    // move up very slowly unitl we have moved 2 inches. Idea is to find minimum
    // power
    // need to move the elevator up, because it is very negatively weighted

    private int mStartPosn = 0;

    private double mPercentOut;

    private PeriodicLogger mPLog;

    public void initMovSlowUp() {
        mStartPosn = getWristPosition();
        mPercentOut = 0.0;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initMovSlowUp"));
    }

    public void execMovSlowUp() {
        mPercentOut += 0.001;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execMovSlowUp"));
    }

    public boolean isCompleteMovSlowUp() {
        int curPosn = getWristPosition();
        boolean isFin = (curPosn - mStartPosn) >= TICKS_PER_DEG * 2;
        if (isFin) {
            mLog.debug(printPosn("isComp") + "\n------------------------------------------------------");
            mLog.debug("Holding at output %.3f", BASE_PERCENT_OUT);
            mPercentOut = BASE_PERCENT_OUT;
            mTalon.set(ControlMode.PercentOutput, mPercentOut);
        }
        return isFin;
    }

    // TEST EXTEND AND RETRACT
    // -----------------------------------------------------

    public void testExtend() {
        mTalon.set(ControlMode.PercentOutput, .2);
      //  mPLog.debug(printPosn("execExtend"));


    }

    public void testRetract() {
        mTalon.set(ControlMode.PercentOutput, -.2);
     //   mPLog.debug(printPosn("execRetract"));

    }

    // ------------------ Move Extend
    // -------------------------------------------------------------

    private NavXSys mNavXSys;

    /**
     * Move up at 0.3 power more than hold
     */
    public void initExtend() {
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        if (m_movePID != null) {
            m_movePID.disable();
        }
        mStartPosn = getWristPosition();
        mPercentOut = BASE_PERCENT_OUT;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initExtend") + "--------------------------------------------------------");
    }

    public static final int TICKS_AT_90 = (int) ((90 - STARTING_ANGLE) * TICKS_PER_DEG);
    public static final double MAX_WRIST_SPEED = 0.4;

    public void execExtend() {
        if (mDontExtend) {
            return;
        }
        double speed = MAX_WRIST_SPEED;
        if (mTalon.getSelectedSensorVelocity() > 0) {
            int currentPosition = getWristPosition();
            int displacement = currentPosition - TICKS_AT_90;
            double displacementAngle = displacement * (1 / TICKS_PER_DEG);
            speed = -MAX_WRIST_SPEED * Math.sin(displacementAngle);
        }
        mPercentOut = BASE_PERCENT_OUT + speed;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execExtend"));
    }

    // ------------------ Move Retract
    // -------------------------------------------------------------

    /**
     * Move down at -0.1 power
     */
    public void initRetract() {
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        if (m_movePID != null) {
            m_movePID.disable();
        }
        mStartPosn = getWristPosition();
        mPercentOut = BASE_PERCENT_OUT;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initRetract"));
    }

    public void execRetract() {
        if (mDontExtend) {
            return;
        }

        double speed = -MAX_WRIST_SPEED;
        if (mTalon.getSelectedSensorVelocity() < 0) {
            int currentPosition = getWristPosition();
            int displacement = currentPosition - TICKS_AT_90;
            double displacementAngle = displacement * (1 / TICKS_PER_DEG);
            speed = MAX_WRIST_SPEED * Math.sin(displacementAngle);
        }
        mPercentOut = BASE_PERCENT_OUT + speed;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execRetract"));
    }

    // ---------------- Wrist Stop
    // Cmd------------------------------------------------------------

    public void stop() {
        mLog.debug("Killing Wrist");
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        mTalon.set(ControlMode.PercentOutput, BASE_PERCENT_OUT);
    }

    // ---------------Wrist Hold
    // Cmd---------------------------------------------------------------------

    public void holdWrist() {
        double wristSpeed = mTalon.getSelectedSensorVelocity(); // ticks per 100 milliseconds
        int currentPosition = getWristPosition();
        int displacement = currentPosition - TICKS_AT_90;
        double displacementAngle = displacement * (1 / TICKS_PER_DEG);
        double speed = Math.sin(90 - displacementAngle);
    }

    // ---------- hold posn PID using the TritonTech PID
    // ----------------------------------

    /**
     * Sensor is on output of gearing (not on motor) Set the tolerance to +/- 10
     * degree
     */
    public void initHoldPosnPID() {

        if (m_holdPID == null) {
            m_PidOutTalon = new PIDOutTalon(mTalon, BASE_PERCENT_OUT, -0.8, 0.8);
            double kP = 0.2 / (10 * TICKS_PER_DEG); // want 20% power when hit tolerance band of 15 degrees
            double kI = 0.0;
            double kD = 0.0;
            double kF = 0.0;
            double periodInSecs = 0.05; // for hold, check every 50 mS is fine
            m_holdPID = new TTPIDController("wristHold", kP, kI, kD, kF, m_PidSourceTalonPW, m_PidOutTalon,
                    periodInSecs);
            m_holdPID.setAbsoluteTolerance(10 * TICKS_PER_DEG); // allow +- 200 units (0.4 inches) on error
        } else {
            m_holdPID.reset();
        }
    }

    /**
     * Hold at the current position
     */
    public void enableHoldPosnPID() {
        int curPosn = mTalon.getSelectedSensorPosition(0);
        enableHoldPosnPID(curPosn);
    }

    /**
     * Do a PID hold at the specified sensor position
     */
    public void enableHoldPosnPID(int targetPosn) {
        if (m_holdPID == null) {
            initHoldPosnPID();
        }
        m_holdPID.reset();
        mLog.debug("WS.enableHoldPosnPID: target: %d    ---------------------", targetPosn);
        mLog.debug(printPosn("WS.enableHoldPosnPID"));
        m_holdPID.setSetpoint(targetPosn);
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
        mTalon.set(ControlMode.PercentOutput, 0.0);
        mLog.debug(printPosn("disableHoldPosnPID"));
    }

    // move to target using PID ---------------------------------------------

    /**
     * Target assumes that the elevator base position is zero Need to adjust for the
     * actual sensor start position
     * 
     * @param targ
     */
    public void initMoveToTarget(WristTarget targ) {
        m_targ = targ;
        if (m_movePID == null) {
            m_PidOutTalon = new PIDOutTalon(mTalon, BASE_PERCENT_OUT, -0.5, 0.5);
            double kP = 0.2 / 500; // want 20% power when hit tolerance band of 500 units (was 0.001)
            double kI = 0.0;
            double kD = 0.0;
            double kF = 0.0;
            double periodInSecs = 0.05; // for hold, check every 50 mS is fine
            m_movePID = new TTPIDController("PID.wristM2Targ", kP, kI, kD, kF, m_PidSourceTalonPW, m_PidOutTalon,
                    periodInSecs);
            m_movePID.setAbsoluteTolerance(10 * TICKS_PER_DEG); // allow +- one inch - then hand over to posn hold to
                                                                // lock //
            // in
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
        m_movePID.setRamp(3 * TICKS_PER_DEG, 5 * TICKS_PER_DEG); // set ramps to 3 inches
        m_movePID.setBasePower(BASE_PERCENT_OUT, 0.05);
        m_movePID.enable();
    }

    /**
     * Dont need to actually do anything here, because the PID if writing to the
     * Talon What we want to do is wait until the PID is close, then use the holdPID
     * to lock in
     */
    public void execMoveToTarget() {
        if (m_movePID.onTarget() && !m_usingHoldPID) {
            mLog.debug("ES.execMoveToTarget: on target  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            mLog.debug(printPosn("ES.execMoveToTarget:"));
            m_movePID.disable();
            enableHoldPosnPID(m_targ.getTicks());
            m_usingHoldPID = true;
        }
    }

    /**
     * Only return true once we have moved to using holdPID and it is on target
     */
    public boolean isMoveToTargetComplete() {
        if (m_usingHoldPID) {
            return m_holdPID.onTarget();
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
        mTalon.set(ControlMode.PercentOutput, 0.0);
        mLog.debug(printPosn("disableMoveToPID"));
    }

}
