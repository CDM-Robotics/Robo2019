package team6072.robo2019.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.kauailabs.navx.frc.AHRS;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PIDController;

import team6072.robo2019.RobotConfig;

/**
 * Implement a drive subsystem for the 2018 robot Two motors per side, driving a
 * single shaft per side Each motor controlled by a CANTalon on the CAN bus
 *
 * To configure Talon Device Ids, need to use the NI web browser RoboRio config
 * Requires IE and Silverlight Connect to robot wifi, then browse to
 * http://roborio-6072-frc.local
 *
 * As at 2018-02-10, the Talons have been given device IDs that match the PDP
 * port they are connected to See RobotConfig for details
 */
public class DriveSys extends Subsystem {

    private static final Logger mLog = LoggerFactory.getLogger(DriveSys.class);

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

    /**
     * When configuring talon, the configXXX() methods have a timeout param.
     * Recommended value when config during init is 10mSec, because each call witll
     * wait to ensure value set correctly During run time, want the value ot be 0 to
     * avoid blocking main thread
     */
    private DriveSys() {
        System.out.println("6072: DriveSys constructor");

        try {
            mLeft_Master = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_MASTER);
            mLeft_Master.setSafetyEnabled(false);
            mLeft_Master.configOpenloopRamp(0.1, 10);
            mLeft_Master.setNeutralMode(NeutralMode.Brake);
            mLeft_Master.setSensorPhase(true);

            mLeft_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_SLAVE0);
            mLeft_Slave0.set(ControlMode.Follower, RobotConfig.DRIVE_LEFT_MASTER);
            mLeft_Slave0.setInverted(false);

            mRight_Master = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_MASTER);
            mRight_Master.setSafetyEnabled(false);
            mRight_Master.configOpenloopRamp(0.1, 10);
            mRight_Master.setNeutralMode(NeutralMode.Brake);

            mRight_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_SLAVE0);
            mRight_Slave0.set(ControlMode.Follower, RobotConfig.DRIVE_RIGHT_MASTER);
            mRight_Slave0.setInverted(false);

            mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);

            mMasterTalons.add(mRight_Master);
            mMasterTalons.add(mLeft_Master);

            mNavX = NavXSys.getInstance().getNavX();
            initYawPID();
            initDrivePID();
        } catch (Exception ex) {
            System.out.println("Exception in DriveSys ctor: " + ex.getMessage() + "\r\n" + ex.getStackTrace());
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
        System.out.println("DriveSys: init default command");
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

    /**
     * Set the quad posn to same as PW posn. Left motor sensor goes -ve when driving
     * forward This should only be called from Robot.Init because of the time delays
     */
    public void setSensorStartPosn() {

        mLeft_Master.getSensorCollection().setPulseWidthPosition(0, 10);
        mRight_Master.getSensorCollection().setPulseWidthPosition(0, 10);
        int leftPosn = mLeft_Master.getSensorCollection().getPulseWidthPosition();
        int rightPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();

        /* mask out overflows, keep bottom 12 bits */
        int leftAbsPosition = leftPosn & 0xFFF;
        int rightAbsPosn = rightPosn & 0xFFF;

        mLeft_Master.setSelectedSensorPosition(-leftAbsPosition, 0, 10);
        mRight_Master.setSelectedSensorPosition(rightAbsPosn, 0, 10);
        // mTalon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
        // setSelected takes time so wait for it to get accurate print
        sleep(100);
    }

    public int getLeftSensPosn() {
        return mLeft_Master.getSensorCollection().getPulseWidthPosition();
    }

    public void arcadeDrive(double mag, double yaw) {
        yaw = yaw * 0.8; // reduce sensitivity on turn
        mRoboDrive.arcadeDrive(-mag, yaw, true);
    }

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
     * @param distInFeet
     */
    public void initDriveDist(double distInFeet) {
        double circumferenceInFeet = Math.PI * 0.5;
        mTargetDist = (int) ((distInFeet / (circumferenceInFeet)) * 4096);
        mStartPosn = mRight_Master.getSensorCollection().getPulseWidthPosition();
        mTargPosn = mStartPosn + mTargetDist;
        System.out.printf(
                "DS.initDriveDist:  distInFeet: %.3f   mTargdist:  %d    mStartPos: %.3f  mTragPosn: %.3f\r\n",
                distInFeet, mTargetDist, mStartPosn, mTargPosn);
        mHitTarg = false;
        mMoveDistLoopCnt = 0;
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
            System.out.printf("DS.driveDist: start: %d   cur: %d   targ: %d   mag: %.3f  yaw: %.3f  \r\n", mStartPosn,
                    curPosn, mTargPosn, mag, yaw);
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
        return mDrivePID.onTarget();
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
        System.out.println("DS.initDrivePID:  ");
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
        System.out.println("DS.initYawPID:  AHRS.SrcType: " + mNavX.getPIDSourceType().name());
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
        mYawPID.setSetpoint(0);
        mYawPID.enable();
    }

}