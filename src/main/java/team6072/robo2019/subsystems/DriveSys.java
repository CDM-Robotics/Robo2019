package team6072.robo2019.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import team6072.robo2019.subsystems.PIDSourceNavX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import team6072.robo2019.commands.objectives.Objective;

/**
 * Implement a drive subsystem for the 2019 robot Two motors per side, driving a
 * single shaft per side Each motor controlled by a CANTalon on the CAN bus
 *
 * As at 2018-02-10, the Talons have been given device IDs that match the PDP
 * port they are connected to See RobotConfig for details
 */
public class DriveSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(DriveSys.class.getName());
    private static final PeriodicLogger mPLog = new PeriodicLogger(mLog, 5);
    // Objective is to have the Talon LEDs flashing green when driving forward, and
    // sensor phase in sync with velocity - use the graphing in the Phoenix Tuner
    // plot to check
    private static boolean LEFT_INVERT = RobotConfig.DRIVE_LEFT_INVERT;
    private static boolean LEFT_SENSPHASE = RobotConfig.DRIVE_LEFT_SENSPHASE;

    private static boolean RIGHT_INVERT = RobotConfig.DRIVE_RIGHT_INVERT;
    private static boolean RIGHT_SENSPHASE = RobotConfig.DRIVE_RIGHT_SENSPHASE;

    private WPI_TalonSRX mLeft_Master;
    private WPI_TalonSRX mLeft_Slave0;
    private WPI_TalonSRX mLeft_Slave1; // only in 2019
    private WPI_TalonSRX mRight_Master;
    private WPI_TalonSRX mRight_Slave0;
    private WPI_TalonSRX mRight_Slave1; // only in 2019

    ArrayList<TalonSRX> mMasterTalons = new ArrayList<TalonSRX>();

    private DifferentialDrive mRoboDrive;

    private boolean m_RoboControl;      // true - RoboLord is controlling drive

    private static DriveSys mInstance;

    public static DriveSys getInstance() {
        if (mInstance == null) {
            mInstance = new DriveSys();
        }
        return mInstance;
    }

    /**
     * When configuring talon, the configXXX() methods have a timeout param.
     * Recommended value when config during init is 10mSec, because each call will
     * wait to ensure value set correctly During run time, want the value to be 0 to
     * avoid blocking main thread
     */
    private DriveSys() {
        mLog.info("DriveSys ctor  ----------------------------------------------");

        try {
            m_RoboControl = false;
            mLeft_Master = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_MASTER);
            // SetInverted is added to decide if motor should spin clockwise or counter
            // clockwise when told to move positive/forward (green LEDs)
            // This will invert the hbridge output but NOT the LEDs.
            mLeft_Master.setInverted(RobotConfig.DRIVE_LEFT_INVERT); // true);
            mLeft_Master.setSensorPhase(RobotConfig.DRIVE_LEFT_SENSPHASE); // false);

            mLeft_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_SLAVE0);
            mLeft_Slave0.follow(mLeft_Master, FollowerType.PercentOutput);
            mLeft_Slave0.setInverted(InvertType.FollowMaster); // follow tested 2-19

            if (RobotConfig.IS_ROBO_2019) {
                mLeft_Slave1 = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_SLAVE1);
                mLeft_Slave1.follow(mLeft_Master, FollowerType.PercentOutput);
                mLeft_Slave1.setInverted(InvertType.OpposeMaster); // oppose tested 2-19
            }

            mRight_Master = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_MASTER);
            mRight_Master.setInverted(RobotConfig.DRIVE_RIGHT_INVERT); // false); // not inverted
            mRight_Master.setSensorPhase(RobotConfig.DRIVE_RIGHT_SENSPHASE); // false);

            mRight_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_SLAVE0);
            mRight_Slave0.follow(mRight_Master, FollowerType.PercentOutput);
            mRight_Slave0.setInverted(InvertType.FollowMaster); // follow tested 2-19

            if (RobotConfig.IS_ROBO_2019) {
                mRight_Slave1 = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_SLAVE1);
                mRight_Slave1.follow(mRight_Master, FollowerType.PercentOutput);
                mRight_Slave1.setInverted(InvertType.FollowMaster); // follow tested 2-19
            }

            mLeft_Master.configOpenloopRamp(0.1, 10);
            mLeft_Master.setNeutralMode(NeutralMode.Brake);
            // nominal outputs can be selected to ensure that any non-zero requested motor
            // output gets promoted to a minimum output. For example, if the nominal forward
            // is set to +0.10 (+10%), then any motor request within (0%, +10%) will be
            // promoted to +10% assuming request is beyond the neutral dead band.
            mLeft_Master.configNominalOutputForward(0.0, 10);
            mLeft_Master.configNominalOutputReverse(-0.0, 10);
            mLeft_Master.configPeakOutputForward(1.0, 10);
            mLeft_Master.configPeakOutputReverse(-1.0, 10);

            // Current Limiting: If enabled, Talon SRX will monitor the supply-current
            // looking for a conditions where current has exceeded the Peak Current for at
            // least Peak Time. If detected, output is reduced until current measurement is
            // at or under Continuous Current.
            mLeft_Master.configPeakCurrentLimit(100, 10);
            mLeft_Master.configPeakCurrentDuration(100);
            mLeft_Master.configContinuousCurrentLimit(50, 10);
            mLeft_Master.enableCurrentLimit(true);

            mRight_Master.configOpenloopRamp(0.1, 10);
            mRight_Master.setNeutralMode(NeutralMode.Brake);

            mRight_Master.configNominalOutputForward(0.0, 10);
            mRight_Master.configNominalOutputReverse(-0.0, 10);
            mRight_Master.configPeakOutputForward(1.0, 10);
            mRight_Master.configPeakOutputReverse(-1.0, 10);

            mRight_Master.configPeakCurrentLimit(100, 10);
            mRight_Master.configPeakCurrentDuration(100);
            mRight_Master.configContinuousCurrentLimit(50, 10);
            mRight_Master.enableCurrentLimit(true);

            // // Set the quadrature encoders to be the source feedback device for the
            // talons
            // mLeft_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
            // mRight_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
            // // config cruise velocity, acceleration
            // mLeft_Master.configMotionCruiseVelocity(1512, kTimeoutMs); // determined with
            // PhoenixTuner, for motor output 99.22%
            // mLeft_Master.configMotionAcceleration(756, kTimeoutMs); // cruise velocity /
            // 2, so it will take 2 econds to reach cruise velocity
            // mRight_Master.configMotionCruiseVelocity(1512, kTimeoutMs); // determined
            // with PhoenixTuner, for motor output 99.22%
            // mRight_Master.configMotionAcceleration(756, kTimeoutMs); // cruise velocity /
            // 2, so it will take 2 seconds to reach cruise velocity

            mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);
            // get rid of nagging message
            mRoboDrive.setSafetyEnabled(true);
            mRoboDrive.setExpiration(2);
            /*
             * From CTRE - WPI drivetrain classes assume left and right are opposite by
             * default. Call this so we can apply + to both sides when moving forward. DO
             * NOT CHANGE
             */
            mRoboDrive.setRightSideInverted(false);

            mMasterTalons.add(mRight_Master);
            mMasterTalons.add(mLeft_Master);

            setSensorStartPosn();

            mNavX = NavXSys.getInstance();
            createDrivePID();
            initTurnPID();
            SmartDashboard.putNumber("DS_kf", kF_drive);
            SmartDashboard.putNumber("DS_kP", kP_drive);
            SmartDashboard.putNumber("DS_kI", kI_drive);
            SmartDashboard.putNumber("DS_kD", kD_drive);
            mLog.info("DriveSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "DriveSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }


    /**
     * Called by RoboLord to get control of the drive system
     */
    public void setRoboControl(boolean giveControl) {
        m_RoboControl = giveControl;
    }


    public void disable() {
        if (mDrivePID != null) {
            mDrivePID.disable();
        }
        mLeft_Master.set(ControlMode.PercentOutput, 0);
        mRight_Master.set(ControlMode.PercentOutput, 0);
    }

    public void feedTalons() {
        mLeft_Master.feed();
        mLeft_Slave0.feed();
        mRight_Master.feed();
        mRight_Slave0.feed();
        if (RobotConfig.IS_ROBO_2019) {
            mLeft_Slave1.feed();
            mRight_Master.feed();
        }
    }

    @Override
    public void initDefaultCommand() {
        mLog.info("DriveSys: init default command empty");
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

    private static String TAL_LEFT = "DSTalLeft";
    private static String TAL_RIGHT = "DSTalRight";

    public String logMotor() {
        double leftPercent = mLeft_Master.getMotorOutputPercent();
        double leftOutVolts = mLeft_Master.getMotorOutputVoltage();
        double leftCurrent = mLeft_Master.getOutputCurrent();
        double rightPercent = mRight_Master.getMotorOutputPercent();
        double rightOutVolts = mRight_Master.getMotorOutputVoltage();
        double rightCurrent = mRight_Master.getOutputCurrent();
        SmartDashboard.putNumber(TAL_LEFT + "_%", leftPercent);
        SmartDashboard.putNumber(TAL_LEFT + "_V", leftOutVolts);
        SmartDashboard.putNumber(TAL_LEFT + "_C", leftCurrent);
        SmartDashboard.putNumber(TAL_RIGHT + "_%", rightPercent);
        SmartDashboard.putNumber(TAL_RIGHT + "_V", rightOutVolts);
        SmartDashboard.putNumber(TAL_RIGHT + "_C", rightCurrent);
        return String.format("DS.motors  LEFT  pc: %.3f  v: %.3f  c: %.3f    RIGHT  c: %.3f  v: %.3f  c: %.3f",
                leftPercent, leftOutVolts, leftCurrent, rightPercent, rightOutVolts, rightCurrent);
    }

    public String logSensors() {
        int leftSelSens = mLeft_Master.getSelectedSensorPosition();
        int rightSelSens = mRight_Master.getSelectedSensorPosition();
        int leftPW = mLeft_Master.getSensorCollection().getPulseWidthPosition();
        int rightPW = mRight_Master.getSensorCollection().getPulseWidthPosition();
        int leftQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
        int rightQuad = mRight_Master.getSensorCollection().getQuadraturePosition();
        return String.format(
                "DS.sensors:  leftSel: %5d  leftPW: %5d  leftQuad: %5d   rightSel: %5d  rightPW: %5d   rightQuad: %5d",
                leftSelSens, leftPW, leftQuad, rightSelSens, rightPW, rightQuad);
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
     * 
     * @param mag - +ve for forward
     * @param yaw - +ve for right
     */
    public void arcadeDrive(double mag, double yaw) {
        yaw = yaw * 0.8; // reduce sensitivity on turn
        mRoboDrive.arcadeDrive(mag, yaw, true);
    }

    // ------ Drive a specified distance  ----------------------------

    private int mTargetDist = 0; // distance to travel in ticks
    private int mStartPosn = 0; // start position in Talon ticks
    private int mTargPosn = 0; // mStartPosn + mTargetDist
    private boolean mHitTarg = false; // set true when we hit target
    private int mMoveDistLoopCnt; // used for debug output

    /**
     * Tell the drive system that we want to drive N feet. Assume that the yaw PID
     * has been initialized already - we may need to set up here later Need to: log
     * our start position calculate how many ticks we need to travel
     * 
     * @param distInInches
     */
    public void initDriveDistPID(double distInInches, double targetYaw) {
        double circumferenceInInches = Math.PI * 6;
        double revsToTarg = distInInches / circumferenceInInches;
        mTargetDist = (int) (revsToTarg * 4096);
        mStartPosn = mRight_Master.getSelectedSensorPosition();
        mTargPosn = mStartPosn + mTargetDist;
        mLog.debug(
                "-------------------------  initDriveDist  ----------------------------------------\n"
                        + "DS.initDriveDist:  distInInches: %.3f   mTargDist:  %d    mStartPos: %d  mTargPosn: %d\n",
                distInInches, mTargetDist, mStartPosn, mTargPosn);
        mLog.debug("DS.initDriveDist: current yaw: %.3f", NavXSys.getInstance().getYawHeading());
        mHitTarg = false;
        mMoveDistLoopCnt = 0;
        mTurnPIDController.reset();
        mTurnPIDController.setSetpoint(targetYaw);
        mTurnPIDController.enable();
        initDrivePID();
        mDrivePID.setSetpoint(mTargPosn);
        mDrivePID.enable();
    }

    /**
     * While we have not completed driving, move forward Need to: check current
     * position decide if have driven far enough if not, keep driving if yes, stop
     */
    public void driveDistPID() {
        int curPosn = mRight_Master.getSelectedSensorPosition();
        double mag = mDrivePIDOut.getVal();
        double yaw = mTurnPIDOut.getVal();
        int error = mTargPosn - curPosn;
        if (Math.abs(error) < (4096 + 2048)) {
            // if within one revolution, lower the speed
            mag = mag / 2;
        }
        double pidErr = mDrivePID.getError();
        SmartDashboard.putNumber("DS_PID_Err", error);
        SmartDashboard.putNumber("DS_PID_PidErr", pidErr);
        if (mMoveDistLoopCnt++ % 5 == 0) {
            mLog.debug("DS.driveDist:   mag: %.3f  yaw: %.3f   cur: %d   err: %d    pidErr: %.3f \n", mag, yaw, curPosn,
                    error, pidErr);
        }
        mag = mag / PID_SCALE;
        mRoboDrive.arcadeDrive(mag, yaw, false);
        mHitTarg = mDrivePID.onTarget();
    }

    /**
     * Called by external system to find out if we have completed driving distance
     * 
     * @return true if have completed driving
     */
    public boolean isDriveDistPIDComplete() {
        if (mDrivePID.onTarget()) {
            arcadeDrive(0, 0);
            int curPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();
            int err = mTargPosn - curPosn;
            double pidErr = mDrivePID.getError();
            SmartDashboard.putNumber("DS_PID_Err", err);
            SmartDashboard.putNumber("DS_PID_PidErr", pidErr);
            mLog.debug(
                    "DS.driveDist complete: targ: %d   cur: %d   err: %d   pidErr: %.3f \n"
                            + "--------------------------------------------------------",
                    mTargPosn, curPosn, err, pidErr);
            mLog.debug(logSensors());
            sleep(100);
            mLog.debug(logSensors());
            return true;
        }
        return false;
    }

    // ---------------- Drive distance PID -------------------------------

    // resources for understanding PID
    // https://www.youtube.com/watch?v=wkfEZmsQqiA
    // http://blog.opticontrols.com/archives/344 -- good intro to PID
    // https://www.controleng.com/articles/feed-forwards-augment-pid-control/ --
    // what the feed forward term does

    // magic values used to initialize the PID controller
    // From the source code for the PIDController:
    // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/main/java/edu/wpi/first/wpilibj/PIDBase.java
    // If a velocity PID controller is being used, the F term should be set to 1
    // over the maximum setpoint for the output. If a position PID controller is
    // being used, the
    // F term should be set*to 1 over the maximum speed for the output measured in
    // setpoint units per this controller's*
    // update period (see the default period in this class's constructor)
    double kF_drive = 1.0; // 3.0; // feed forward - tries to estimate target and set motor to correct
                           // value
    double kP_drive = 4.0; // 0.5; // specify the proportional (fixed) response to error
    double kI_drive = 0.0; // 0.2; // specify response based on how big error is - larger error means
                           // bigger response
    double kD_drive = 0.4; // 0.1; // rarely used - specify response based on how fast error is changing

    double PID_SCALE = 1000;

    // calculate allowed dist error in encoder ticks
    private static final int ALLOWED_DISTERR = (int) (3 / (Math.PI * 6) * 4096);

    private PIDController mDrivePID;
    private PIDOutReceiver mDrivePIDOut;
    private WPIPIDSourceTalon mTalonPIDSource;

    /**
     * Create the drive PID controller. Should only be done once
     */
    private void createDrivePID() {
        mDrivePIDOut = new PIDOutReceiver();
        mTalonPIDSource = new WPIPIDSourceTalon(mRight_Master);
        mDrivePID = new PIDController(kP_drive, kI_drive, kD_drive, kF_drive, mTalonPIDSource, mDrivePIDOut);
        mDrivePID.setName("DS.DrivePID");

        mDrivePID.setOutputRange(0 - PID_SCALE, PID_SCALE);
        // Makes PIDController.onTarget() return True when PIDInput is within the
        // Setpoint +/- the absolute tolerance.
        mDrivePID.setAbsoluteTolerance(ALLOWED_DISTERR);
        mDrivePID.setContinuous(false);
    }

    /**
     * Initialize the drive PID for a new distance run
     */
    private void initDrivePID() {
        kF_drive = SmartDashboard.getNumber("DS_kf", 0.0);
        kP_drive = SmartDashboard.getNumber("DS_kP", 0.0);
        kI_drive = SmartDashboard.getNumber("DS_kI", 0.0);
        kD_drive = SmartDashboard.getNumber("DS_kD", 0.00);
        mDrivePID.reset();
        mDrivePID.setPID(kP_drive, kI_drive, kD_drive, kF_drive);
        mLog.info("DS.initDrivePID:  kf: %.3f  kP: %.3f  kI: %.3f  kD: %.3f", kF_drive, kP_drive, kI_drive, kD_drive);
    }

    // ------------------ NavX PID for yaw ------------------------------------

    // static final double kF_yaw = 0.00;
    // static final double kP_yaw = 0.03;
    // static final double kI_yaw = 0.00;
    // static final double kD_yaw = 0.00;
    // /* This tuning parameter indicates how close to "on target" the */
    // /* PID Controller will attempt to get. */
    // static final double kToleranceDegrees = 2.0f;


    // private PIDController mYawPID;
    // private PIDOutReceiver mYawPIDOut;

    // private void initYawPID() {
    //     mLog.info("DS.initYawPID:  AHRS.SrcType: " + mNavX.getNavX().getPIDSourceType().name());
    //     mYawPIDOut = new PIDOutReceiver();
    //     mYawPID = new PIDController(kP_yaw, kI_yaw, kD_yaw, kF_yaw, mNavX.getNavX(), mYawPIDOut);
    //     mYawPID.setName("DS.YawPID");
    //     mYawPID.setInputRange(-180.0f, 180.0f);
    //     mYawPID.setOutputRange(-1.0, 1.0);
    //     // Makes PIDController.onTarget() return True when PIDInput is within the
    //     // Setpoint +/- the absolute tolerance.
    //     mYawPID.setAbsoluteTolerance(kToleranceDegrees);
    //     // Treats the input ranges as the same, continuous point rather than two
    //     // boundaries, so it can calculate shorter routes.
    //     // For example, in a gyro, 0 and 360 are the same point, and should be
    //     // continuous. Needs setInputRanges.
    //     mYawPID.setContinuous(true);
    //     // mYawPID.setSetpoint(0);
    //     // mYawPID.enable();
    // }

// 
    // AlignmentTurnCmd -------------------------------------------------

    static final double mKP_turn = 2.0;
    static final double mKI_turn = 0.0;
    static final double mKD_turn = 1.0;
    static final double mKF_turn = 0.0;
    /* This tuning parameter indicates how close to "on target" the */
    /* PID Controller will attempt to get. */
    static final double mKToleranceDegreesTurn = 2.0;

    private NavXSys mNavX;
    private PIDController mTurnPIDController;
    private PIDOutReceiver mTurnPIDOut;
    private PIDSourceNavX mTurnPIDSource;

    private double mTurnDriveSpeed;

    public void initTurnPID() {
        mNavX.zeroYawHeading();
        mLog.info("DS.initYawPID:  CurrentYaw: " + mNavX.getYawHeading());
        mTurnPIDOut = new PIDOutReceiver();
        mTurnPIDSource = new PIDSourceNavX();
        mTurnPIDController = new PIDController(mKP_turn, mKI_turn, mKD_turn, mKF_turn, mTurnPIDSource, mTurnPIDOut);
        mTurnPIDController.setName("DS.TurnPID");
        mTurnPIDController.setInputRange(-180.0, 180.0);
        mTurnPIDController.setOutputRange(-100.0, 100.0);
        // Makes PIDController.onTarget() return True when PIDInput is within the
        // Setpoint +/- the absolute tolerance.
        mTurnPIDController.setAbsoluteTolerance(mKToleranceDegreesTurn);
        // Treats the input ranges as the same, continuous point rather than two
        // boundaries, so it can calculate shorter routes.
        // For example, in a gyro, 0 and 360 are the same point, and should be
        // continuous. Needs setInputRanges.
        mTurnPIDController.setContinuous(true);

    }


    private double mTurnPIDSetpoint;
    private boolean mTurnDrivePIDEnabled = false;

    /**
     * Set a target for the turn, and a drive speed to be using
     */
    public void initTurnDrivePID(double targetYaw, double turnDriveSpeed) {
        mLog.debug("DS turn Target : " + targetYaw + " Current Yaw : " + mNavX.getYawHeading());
        mTurnDriveSpeed = turnDriveSpeed;
        mTurnPIDSetpoint = targetYaw;
        mTurnPIDController.setSetpoint(targetYaw);
        mTurnDrivePIDEnabled = true;
        mTurnPIDController.enable();
    }

    /**
     * The yaw from turnPIDOut is not an angle, it is a arcade drive value (-1 to +1) possibly scaled
     * Allow updating the setpoint for the PID
     */
    public void execTurnDrivePID(double setPoint) {
        if (!mTurnDrivePIDEnabled) {
            return;
        }
        if (setPoint != mTurnPIDSetpoint) {
            mTurnPIDSetpoint = setPoint;
            mTurnPIDController.setSetpoint(setPoint);
        }
        double yaw = mTurnPIDOut.getVal() / 100;
        if (abs(yaw) < .4) {
            yaw = (yaw / yaw) * .4;
        }
        mPLog.debug("DS : Yaw Magnitude = " + yaw + "   Yaw Heading : " + mNavX.getYawHeading());
        mRoboDrive.arcadeDrive(mTurnDriveSpeed, yaw, true);
    }


    public boolean isFinishedTurnDrivePID() {
        boolean finished = false;
        if (mTurnPIDController.onTarget()) {
            mLog.debug("DS TurnPID complete   Current Yaw : " + mNavX.getYawHeading());
            finished = true;
            arcadeDrive(0, 0);
            mTurnPIDController.disable();
        }
        return finished;
    }


    /**
     * Disable the turnPID process - used by RoboLord when on centerline
     */
    public void stopTurnDrivePID() {
        mTurnDrivePIDEnabled = false;
        mTurnPIDController.disable();
    }

    public double abs(double num) {
        if (num < 0) {
            num = num * -1;
        }
        return num;
    }

}