
package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import team6072.robo2019.pid.TTPIDController;

/**
 * Add your docs here.
 */
public class ElevatorSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(ElevatorSys.class.getName());

    private static ElevatorSys mInstance;

    public static enum Direction {
        Up, Down
    }


    /**
     * How many sensor units per rotation.
     * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
     */
    private static final int kCTREUnitsPerRotation = 4096; // 4096;

    private static final int kUnitsPerRotation = kCTREUnitsPerRotation;

    // inches of elevator travel per complete rotation of encoder  gear is 1 inch diameter
    private static final double kDistancePerRotation = 1.75 * Math.PI;

    private static final int kUnitsPerInch = (int) Math.round(kUnitsPerRotation / kDistancePerRotation);

    // talon setup
    // -------------------------------------------------------------------------------

    private WPI_TalonSRX mTalon;

    private static final boolean TALON_INVERT = false;
    // The sensor position must move in a positive direction as the motor controller
    // drives positive output (and LEDs are green)
    // true inverts the sensor
    private static final boolean TALON_SENSOR_PHASE = false;

    private static final int TALON_FORWARD_LIMIT = -1;

    private static final int TALON_REVERSE_LIMIT = -1;

    /*
     * set the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    private static final int TALON_ALLOWED_CLOSELOOP_ERROR = 0;

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

    /**
     * Log the sensor position at power up - use this as the base reference for positioning.
     */
    private int mBasePosn;

    
    
    public static ElevatorSys getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSys();
        }
        return mInstance;
    }



    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new ElvMoveUpSlow());
    }


    public ElevatorSys() {
        mLog.info("ElevatorSys ctor  ----------------------------------------------");
        try {
            mTalon = new WPI_TalonSRX(RobotConfig.ELEVATOR_TALON);
            mTalon.setSafetyEnabled(false);
            mTalon.configFactoryDefault();
            mTalon.setName(String.format("%d: Elevator", RobotConfig.ELEVATOR_TALON));
            // in case we are in magic motion or position hold mode
            mTalon.set(ControlMode.PercentOutput, 0);

            mTalon.setSensorPhase(TALON_SENSOR_PHASE);
            mTalon.setInverted(TALON_INVERT);
            mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
            mTalon.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

            // mTalon.configForwardSoftLimitThreshold(TALON_FORWARD_LIMIT, kTimeoutMs);
            // mTalon.configForwardSoftLimitEnable(false, kTimeoutMs);
            // mTalon.configReverseSoftLimitThreshold(TALON_REVERSE_LIMIT, kTimeoutMs);
            // mTalon.configReverseSoftLimitEnable(false, kTimeoutMs);

            mTalon.configOpenloopRamp(0.1, kTimeoutMs);
            mTalon.setNeutralMode(NeutralMode.Brake);

            // see setup for motion magic in s/w manual 12.6
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
            mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);

            // set up current limits
            mTalon.configContinuousCurrentLimit(30, kTimeoutMs);
            mTalon.configPeakCurrentLimit(40, kTimeoutMs);
            mTalon.configPeakCurrentDuration(200, kTimeoutMs);
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

            m_PidSourceTalonPW = new PIDSourceTalonPW(mTalon, 0);

            setSensorStartPosn();

            mLog.info("ElevatorSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "ElevatorSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }


    /**
     * Ensure we are not in MotionMagic or PositionHold mode
     */
    public void resetTalon() {
        mTalon.set(ControlMode.PercentOutput, 0);
    }

    // grab the 360 degree position of the MagEncoder's absolute position, and set
    // the relative sensor to match.
    // should only be called on robot.init
    public void setSensorStartPosn() {
        mTalon.getSensorCollection().setPulseWidthPosition(0, kTimeoutMs);
        //mBasePosn = mTalon.getSensorCollection().getPulseWidthPosition();
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
        int quadPosn = mTalon.getSensorCollection().getQuadraturePosition();
        int pwPosn = mTalon.getSensorCollection().getPulseWidthPosition();
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
        double closedLoopErr = mTalon.getClosedLoopError(0);

        mLastQuadPosn = quadPosn;
        return String.format("ES.%s  base: %d  selPosn: %d  vel: %.3f  pcOut: %.3f  volts: %.3f  cur: %.3f", 
                caller, mBasePosn, selSensPosn, vel, mout, voltOut, curOut);
        // System.out.println("ArmSys." + caller + ": topSwitch: " + mTopCounter.get() +
        // " botSwitch: " + mBotCounter.get());
        // System.out.println("ArmSys." + caller + ": Vel: " + vel + " pwVel: " + pwVel
        // + " MotorOut: " + mout + " voltOut: " + voltOut+ " clErr: " + closedLoopErr);
        // System.out.println("ES." + caller + ":  base: " + mBasePosn + "  sens: " + sensPosnSign + absSensPosn
        //         + "  rDelta: " + relDelta + "  quad: " + quadPosn + "  qDelta: " + quadDelta + "  pw: " + pwPosn
        //         + "  clErr: " + closedLoopErr);

        // System.out.println(" out%: " + mout + " volt: " + voltOut + " curOut: " + curOut);
        // shuffleBd();
    }
    

    // MovSlowUpCmd  --------------------------------------------------------

    private int mStartPosn = 0;

    private int mCurPosn = 0;

    private double mPercentOut;

    private PeriodicLogger mPLog;


    public void initMovSlowUp() {
        mStartPosn =  mTalon.getSensorCollection().getPulseWidthPosition();
        mPercentOut = 0.0;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initMovSlowUp"));
    }


    public void execMovSlowUp() {
        mPercentOut += 0.001;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("initMovSlowUp"));
    }


    public boolean isCompleteMovSlowUp() {
        int curPosn = mTalon.getSensorCollection().getPulseWidthPosition();
        boolean isFin = (curPosn - mStartPosn) >= 500;
        if (isFin) {
            mLog.debug(printPosn("isComp") + "\n------------------------------------------------------");
            mLog.debug("Holding at output 0.18");
            mPercentOut = 0.19;
            mTalon.set(ControlMode.PercentOutput, mPercentOut);
        }
        return isFin;
    }
    

    // -------------------------  basic hold  -------------------------------------

    // a motor output of BASE_POWER holds the motor in place when not disturbed
    private static final double BASE_PERCENT_OUT = 0.2;

    public void holdPosnPower() {
        mLog.debug(printPosn("holdPosnPower") + "\n------------------------------------------------------");
        mLog.debug("Holding at output %.3f", BASE_PERCENT_OUT);
        mPercentOut = BASE_PERCENT_OUT;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
    }


    // ------------------ Move Up  -------------------------------------------------------------

    public void initMoveUp() {
        mStartPosn = mTalon.getSensorCollection().getPulseWidthPosition();
        mPercentOut = 0.0;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initMoveUp") + "--------------------------------------------------------");
    }

    public void execMoveUp() {
        mPercentOut = 0.4;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execMoveUp"));
    }


    // ------------------ Move Down  -------------------------------------------------------------

    public void initMoveDown() {
        mStartPosn = mTalon.getSensorCollection().getPulseWidthPosition();
        mPercentOut = 0.0;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initMoveDown"));
    }

    public void execMoveDown() {
        mPercentOut = -0.1;
        mTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execMoveDown"));
    }




    // ------------------ Magic motion drive distance   -------------------------------------------

    // Motion Magic values
    /*
     * kF calculation: = full forward value * duty-cycle (%) / runtime calculated
     * target (ticks, velocity units/100 ms) = 1023 * 100% / 1525 =
     * 0.67081967213114754098360655737705 (1525 determined through PhoenixTuner
     * self-test)
     */
    private static final double kF = 0.67081967213114754098360655737705;
    private static final double kP = 1;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 5;
    private static final int kSlotIdx = 0;

    private double mMMTarget; // target in encoder ticks

    private int mTargetDist;

    public void initMagicMotion(double distInInches) {
        double circumferenceInInches = Math.PI * 6;
        double revsToTarg = distInInches / circumferenceInInches;
        mTargetDist = (int) (revsToTarg * 4096);
        mStartPosn = mTalon.getSelectedSensorPosition();
        mMMTarget = mStartPosn + mTargetDist;

        // select profile slot
        mTalon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
        // config pidf values
        mTalon.config_kF(kSlotIdx, kF, kTimeoutMs);
        mTalon.config_kP(kSlotIdx, kP, kTimeoutMs);
        mTalon.config_kI(kSlotIdx, kI, kTimeoutMs);
        mTalon.config_kD(kSlotIdx, kD, kTimeoutMs);

        mTalon.setSelectedSensorPosition(0);
        mTalon.setSelectedSensorPosition(0);;
    }

    /**
     * Called by the command exec - test if in target tolerance and return true
     * 
     * @param target
     * @return
     */
    public boolean motionMagicOnTarget() {
        double tolerance = 50;
        double currentPos = mTalon.getSelectedSensorPosition();
        return Math.abs(currentPos - mMMTarget) < tolerance;
    }


    // -----------------------------  hold position  ---------------------------------------



    public void initHoldPosnPID() {

    }

    /**
     * Switch to using closed loop position hold at current position
     */
    public void holdPosnPID() {
        // select the hold PID slot
        mTalon.selectProfileSlot(kPIDSlot_Hold, 0);
        // init PID for hold
        mTalon.configAllowableClosedloopError(kPIDSlot_Hold, TALON_ALLOWED_CLOSELOOP_ERROR, kTimeoutMs);
        // a motor output of BASE_POWER holds the motor in place when not disturbed
        double kF = BASE_PERCENT_OUT * 1023 / 4096;
        mTalon.config_kF(kPIDSlot_Hold, kF, kTimeoutMs); // normally 0 for position hold but putting in small 0.1 dampsoscillation
        double kP = 0.4 * 1023 / 4096;
        mTalon.config_kP(kPIDSlot_Hold, kP, kTimeoutMs); // kP 1.0 used on elevator 2018-02-17
        mTalon.config_kI(kPIDSlot_Hold, 0.0, kTimeoutMs);
        mTalon.config_kD(kPIDSlot_Hold, 0.0, kTimeoutMs);

        double curPosn = mTalon.getSelectedSensorPosition(0);
        // double curPosn = mTalon.getSensorCollection().getPulseWidthPosition();
        // double curPosn = holdPos;
        mLog.debug(printPosn("holdPosnPID") + "------------------------------------------");
        int loopCnt = 0;
        // In Position mode, output value is in encoder ticks or an analog value,
        // depending on the sensor.
        mTalon.set(ControlMode.Position, curPosn);
    }


    // ----------  PID using the local WPI PID  ----------------------------------

    private TTPIDController m_holdPID;
    private PIDSourceTalonPW m_PidSourceTalonPW;
    private PIDOutTalon m_PidOutTalon;

    /**
     * Sensor is on output of gearing (not on motor)
     * gear dia = 1.5" so  1 rev = 1.5 * 3.14159 = 4.712" (theoretical)
     * but the elevator has gearing for the carriage
     * MEASURED ticks per inch is 535
     * an error of 100 ticks = 0.19 inches
     */
    public void initHoldPosnTTPID() {

        m_PidOutTalon = new PIDOutTalon(mTalon, BASE_PERCENT_OUT, -0.8, 0.8);
        double kP = 0.2/500;              // want 20% power when hit tolerance band of 500 units (was 0.001)
        double kI = 0.0;
        double kD = 0.0;
        double kF = 0.0;
        double periodInSecs = 0.05;     // for hold, check every 50 mS is fine
        m_holdPID = new TTPIDController("elvHold", kP, kI, kD, kF, m_PidSourceTalonPW, m_PidOutTalon, periodInSecs);
        m_holdPID.setAbsoluteTolerance(200);       // allow +- 200 units (0.4 inches) on error 
    }


    public void enableHoldPosnTTPID() {
        int curPosn = mTalon.getSelectedSensorPosition(0);
        enableHoldPosnTTPID(curPosn);
    }

    /**
     * Do a PID hold at the specified sensor position
     */
    public void enableHoldPosnTTPID(int posn) {
        if (m_holdPID == null) {
            initHoldPosnTTPID();
        }
        m_holdPID.reset();
        mLog.debug("enableHoldPosnTTPID: posn: %d    ---------------------", posn);
        mLog.debug(printPosn("enableHoldPosnTTPID"));
        m_holdPID.setSetpoint(posn);
        m_holdPID.enable();
    }


    /**
     * Disable the hold PID. This will send 0 to the PID out, which writes to the talon
     */
    public void disableHoldPosnTTPID() {
        if (m_holdPID != null) {
            m_holdPID.disable();
        }
        mTalon.set(ControlMode.PercentOutput, 0.0);
        mLog.debug(printPosn("disableHoldPosnTTPID"));
    }


    // move to target using PID  ---------------------------------------------


    private static final double GEAR_DIA_INCHES = 1.5;
    
    // MEASURE the ticks per inch on physical mechanism
    private static final int TICKS_PER_INCH = 370;      // MEASURED
    private static final double INCHES_PER_REVOLUTION = 4096 / TICKS_PER_INCH;

    private static final double ELEVATOR_FLOOR_INCHES = 13.0; // inches from ground when elevator at zero

    private static final double ROCKET_HATCH_LO_INCHES = ((12 + 7) - ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_HATCH_LO = (int) (ROCKET_HATCH_LO_INCHES * TICKS_PER_INCH);

    private static final double ROCKET_HATCH_MID_INCHES = (ROCKET_HATCH_LO_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_MID = 5000; //(int) (ROCKET_HATCH_MID_INCHES * TICKS_PER_INCH);

    private static final double ROCKET_HATCH_HI_INCHES = (ROCKET_HATCH_MID_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_HI = (int) (ROCKET_HATCH_HI_INCHES * TICKS_PER_INCH);

    public enum Target {
        RocketHatchHi(ROCKET_HATCH_HI), 
        RocketHatchMid(ROCKET_HATCH_MID), 
        RocketHatchLo(ROCKET_HATCH_LO);
        private int mTicks;

        Target(int ticks) {
            mTicks = ticks;
        }

        public int getTicks() {
            return mTicks;
        }
    }
    
    private Target m_targ;
    private TTPIDController m_movePID;

    private boolean m_usingHoldPID;


    /**
     * Target assumes that the elevator base position is zero
     * Need to adjust for the actual sensor start position
     * @param targ
     */
    public void initMoveToTarget(Target targ) {
        m_targ = targ;
        m_PidOutTalon = new PIDOutTalon(mTalon, BASE_PERCENT_OUT, -0.8, 0.8);
        double kP = 0.2 / 500; // want 20% power when hit tolerance band of 500 units (was 0.001)
        double kI = 0.0;
        double kD = 0.0;
        double kF = 0.0;
        double periodInSecs = 0.05; // for hold, check every 50 mS is fine
        m_movePID = new TTPIDController("elvTarg", kP, kI, kD, kF, m_PidSourceTalonPW, m_PidOutTalon, periodInSecs);
        m_movePID.setAbsoluteTolerance(4096); // allow +- full revolution for on target - then hand over to posn hold to lock in
        int curPosn = mTalon.getSelectedSensorPosition(0);
        int calcTarg = targ.getTicks();
        mLog.debug("initMoveToTarget: curPos: %d    targ: %s(%d)  calcTarg: %d  ---------------------", 
                        curPosn, m_targ.toString(),m_targ.getTicks(), calcTarg);
        mLog.debug(printPosn("initMoveToTarget"));
        m_usingHoldPID = false;
        m_movePID.setSetpoint(calcTarg);
        m_movePID.setRamp(3 * TICKS_PER_INCH, 3 * TICKS_PER_INCH);      // set ramps to 3 inches
        m_movePID.enable();
    }

    /**
     * Dont need to actually do anything here, because the PI if writign to the Talon
     * What we want to do is wait until the PID is close, then use the holdPID to lock in
     */
    public void execMoveToTarget() {
        if (m_movePID.onTarget()) {
            mLog.debug("ES.execMoveToTarget: on target");
            mLog.debug(printPosn("ES.execMoveToTarget:"));
            m_movePID.disable();
            enableHoldPosnTTPID(m_targ.getTicks());
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
