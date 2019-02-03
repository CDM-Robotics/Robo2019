
package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.commands.elevator.ElvMoveUpSlow;
import team6072.robo2019.logging.*;

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
    private static final boolean TALON_SENSOR_PHASE = true;

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
    public static final int kPIDLoopIdx = 0;

    // paramter to the configXXX() methods. Set to non-zero to have talon wait to
    // check and report error
    public static final int kTimeoutMs = 10;

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
        setDefaultCommand(new ElvMoveUpSlow());
    }


    public ElevatorSys() {
        mLog.info("ElevatorSys ctor  ----------------------------------------------");
        try {
            mTalon = new WPI_TalonSRX(RobotConfig.ELEVATOR_TALON);
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

            // the nominal values are used in closed loop when in deadband
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
        mBasePosn = mTalon.getSensorCollection().getPulseWidthPosition();
        int absolutePosition = mBasePosn;
        /* mask out overflows, keep bottom 12 bits */
        absolutePosition &= 0xFFF;
        if (TALON_SENSOR_PHASE)
            absolutePosition *= -1;
        if (TALON_INVERT)
            absolutePosition *= -1;
        /* set the quadrature (relative) sensor to match absolute */
        mTalon.setSelectedSensorPosition(absolutePosition, 0, kTimeoutMs);
        printPosn("setStart");
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
        return String.format("ES.%s  pwPosn: %d  vel: %.3f  pcOut: %.3f  volts: %.3f  cur: %.3f", 
                caller, pwPosn, vel, mout, voltOut, curOut);
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
        }
        return isFin;
    }

}
