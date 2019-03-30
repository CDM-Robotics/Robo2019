
package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import team6072.robo2019.pid.PIDSourceNavXPitch;
import team6072.robo2019.pid.TTPIDController;




public class ClimberSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(DistanceMeasureSys.class.getName());


    private static ClimberSys mInstance;

    public static ClimberSys getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberSys();
        }
        return mInstance;
    }


    private NavXSys mNavX;
    private ElevatorSys mElvSys;
    private WristSys mWristSys;

    private WPI_TalonSRX mClimbTalon;

    private WPI_TalonSRX mElvTalon;

    private static final boolean TALON_INVERT = false;
    private static final boolean TALON_SENSOR_PHASE = false;
    
    public static final int kTimeoutMs = 10;
    public static final int kPIDLoopIdx = 0;


   
    public static final double BASE_PERCENT_OUT = RobotConfig.CLIMB_BASE_PERCENT_OUT;
    /**
     * The climber sys has to:
     *      move the intake to hab position and do pid hold
     *      move elevator to hab position
     *      start moving elevator and climber in sync
     *      use NavX pitch to detect how horzontal we a
     */
    private ClimberSys() {
        mLog.info("ClimberSys ctor  ----------------------------------------------");
        try {
            mNavX = NavXSys.getInstance();
            mElvSys = ElevatorSys.getInstance();
            mWristSys = WristSys.getInstance();

            mClimbTalon = new WPI_TalonSRX(RobotConfig.WRIST_MASTER);
            mClimbTalon.configFactoryDefault();
            mClimbTalon.setName(String.format("Wrist: %d", RobotConfig.WRIST_MASTER));
            // in case we are in magic motion or position hold model 
            mClimbTalon.set(ControlMode.PercentOutput, 0);

            mClimbTalon.setSensorPhase(TALON_SENSOR_PHASE);
            mClimbTalon.setInverted(TALON_INVERT);

            mClimbTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
            mClimbTalon.setSelectedSensorPosition(0);

            m_PidOutTalon = new PIDOutTalon(mElvTalon, mElvSys.BASE_PERCENT_OUT, -0.8, 0.8);
            double kP = 0.2 / 10; // 20% power when 10 degres from set point
            double kI = 0.0;
            double kD = 0.0;
            double kF = 0.0;
            double periodInSecs = 0.05; // for hold, check every 50 mS is fine
            m_holdPID = new TTPIDController("climb", kP, kI, kD, kF, m_NavXSource, m_PidOutTalon, periodInSecs);
            m_holdPID.setAbsoluteTolerance(5); // allow +- 5 degrees

            mLog.info("ClimberSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "ClimberSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }


    @Override
    public void initDefaultCommand() {
    }


    private TTPIDController m_holdPID;
    private PIDSourceNavXPitch m_NavXSource;
    private PIDOutTalon m_PidOutTalon;

    /**
     * When climbing, want to maintain pitch from 0 to -10 degress (i.e. nose down)
     * Set climb going up steadliy, and modify elevator to hold pitch steady PID
     * with NavX as sensor, and elevator talon as target
     * 
     * When climb talon has hit MAX ticks, we want to fall forward onto platform
     */
    public void initClimb() {
        mElvSys.disable(); // stop elevator sys from doing anything
        mWristSys.disable(); // stop wrist sys from doing anything
        m_holdPID.enable();
        mClimbTalon.set(ControlMode.PercentOutput, 0.1);
    }

    public String printPosn(String caller) {
        int sensPosn = mClimbTalon.getSelectedSensorPosition(0);
        String sensPosnSign = "(+)";
        int absSensPosn = Math.abs(sensPosn);

        return String.format(
                "ES.%s  base: %d  selPosn: %d  cur: %.3f",
                caller, mStartPosn, sensPosn,
                mPercentOut);
    }

    private int mStartPosn;

    private double mPercentOut;

    private PeriodicLogger mPLog;

    private static final int HAB_LEVEL = 123456; // climb ticks when at hab level
    private static final int MAX_CLIMB = 500; // when we want to stop the climber

    public void initPushDownSlow() {
        mStartPosn = mClimbTalon.getSelectedSensorPosition();
        mPercentOut = 0.0;
        mClimbTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog = new PeriodicLogger(mLog, 5);
        mLog.debug(printPosn("initPushDownSlow"));
    }

    public void execPushDownSlow() {
        mPercentOut += 0.001;
        mClimbTalon.set(ControlMode.PercentOutput, mPercentOut);
        mPLog.debug(printPosn("execMovSlowUp"));
    }

    public boolean isCompletePushDownSlow() {
        int curPosn = mClimbTalon.getSelectedSensorPosition();
        boolean isFin = (curPosn - mStartPosn) >= MAX_CLIMB;
        if (isFin) {
            mLog.debug(printPosn("isComp") + "\n------------------------------------------------------");
            mPercentOut = BASE_PERCENT_OUT;
            mLog.debug("Holding at output %.3f", BASE_PERCENT_OUT);
            mClimbTalon.set(ControlMode.PercentOutput, mPercentOut);
        }
        return isFin;
    }

    // ------------------------- basic hold -------------------------------------

    public void holdPosnPower() {
        mLog.debug(printPosn("holdPosnPower") + "\n------------------------------------------------------");
        mLog.debug("Holding at output %.3f", BASE_PERCENT_OUT);
        mPercentOut = BASE_PERCENT_OUT;
        mClimbTalon.set(ControlMode.PercentOutput, mPercentOut);
    }
    /**
     * When we get to HAB_LEVEL, stop moving the elvator but continue the climber
     * so that the robot tips forward.
     */
    public void execClimb() {

    }

    
    /**
     * Climb is completed when the climb talon hits MAX ticks
     */
    public boolean climbCompleted() {
        return false;
    }


}
