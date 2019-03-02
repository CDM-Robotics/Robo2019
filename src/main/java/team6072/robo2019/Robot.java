// mission failed, we'll get em next time
package team6072.robo2019;

import java.io.*;
import java.util.logging.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team6072.robo2019.logging.*;
import team6072.robo2019.commands.drive.ArcadeDriveCmd;
import team6072.robo2019.commands.drive.DriveDistCmd;
import team6072.robo2019.commands.elevator.ElvMoveUpSlow;
import team6072.robo2019.commands.pneumatics.FlowerCloseCmd;
import team6072.robo2019.commands.pneumatics.FlowerOpenCmd;
import team6072.robo2019.device.DistanceSensor;
import team6072.robo2019.subsystems.*;

public class Robot extends TimedRobot {

    // define the logger for this class. This should be done for every class
    private static LogWrapper mLog;
    private PeriodicLogger mLogPeriodic;

    private ControlBoard mControlBoard;

    private DriveSys mDriveSys;
    private ElevatorSys mElvSys;
    private PneumaticSys mPneuSys;
    private NavXSys mNavXsys;
    private WristSys mWristSys;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            File dir = Filesystem.getDeployDirectory();
            String logFile = "NOTFOUND";
            if (dir.isDirectory()) {
                logFile = dir.getAbsolutePath() + "/logging.properties";
            }
            System.out.println("**********  logConfig: " + logFile + "  *********************");
            // BufferedReader br = new BufferedReader(new FileReader(logFile));
            // String line = null;
            // while ((line = br.readLine()) != null) {
            // System.out.println(line);
            // }
            // br.close();
            FileInputStream configFile = new FileInputStream(logFile);
            LogManager.getLogManager().readConfiguration(configFile);
        } catch (IOException ex) {
            System.out.println("WARNING: Could not open configuration file");
            System.out.println("WARNING: Logging not configured (console output only)");
        }
        mLog = new LogWrapper(Robot.class.getName());
        mLogPeriodic = new PeriodicLogger(mLog, 50);
        try {
            if (RobotConfig.IS_ROBO_2019) {
                mLog.info("robotInit: -----------------    2019    -----------------------     2019    2019    2019");
            } else {
                mLog.info("robotInit: -----------------    2018    -----------------------     2018    2018    2018");
            }

            mControlBoard = ControlBoard.getInstance();
            mDriveSys = DriveSys.getInstance();
            mElvSys = ElevatorSys.getInstance();
            mNavXsys = NavXSys.getInstance();
            mPneuSys = PneumaticSys.getInstance();
            mWristSys = WristSys.getInstance();

            // CameraServer.getInstance().startAutomaticCapture();

            NetworkTableInstance tblInst = NetworkTableInstance.getDefault();
            tblInst.setUpdateRate(0.01); // tell network tables to update every 10 mSec
            NetworkTable tbl = tblInst.getTable("Vision_Drive");
            NetworkTableEntry ent = tbl.getEntry("CamName");
            ent.setString("Initial test from robo");

            mLog.info("robotInit: Completed   ---------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "Robot.robotInit:  exception: " + ex.getMessage());
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
        mLog.info("Robot.disabledInit  ----------------------------");
        Scheduler.getInstance().removeAll();
        if (mDriveSys != null) {
            mDriveSys.disable();
        }
        if (mElvSys != null) {
            mElvSys.disable();
        }
        if (mDistSens != null) {
            mDistSens.disable();
        }
        if (mWristSys != null) {
            mWristSys.disable();
        }
    }

    /**
     * gets called every 20 mSec when disabled
     */
    @Override
    public void disabledPeriodic() {
        // mDSSensors.debug(mDriveSys.logSensors());
        // mLogPeriodic.debug(mElvSys.printPosn("disPer:"));
    }

    // *********************** Autonomous
    // *********************************************************

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        try {
            mLog.info("autonomousInit: -----------------------------");
            NavXSys.getInstance().zeroYawHeading();
            Scheduler.getInstance().removeAll();
            mArcadeDriveCmd = new ArcadeDriveCmd(mControlBoard.mDriveStick);
            Scheduler.getInstance().add(new FlowerOpenCmd());
            Scheduler.getInstance().add(mArcadeDriveCmd);
        } catch (Exception ex) {
            mLog.severe(ex, "Robot.autoInit:  exception: " + ex.toString());
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        // mLogPeriodic.debug(mDriveSys.logSensors());
    }

    // *********************** Teleop
    // *********************************************************

    ArcadeDriveCmd mArcadeDriveCmd;
    DriveDistCmd mDriveDistCmd;
    ElvMoveUpSlow mElvSlowCmd;

    DigitalInput mHallSwitch;
    Counter mHallCtr;

    DistanceSensor mDistSens;

    private int m_teleopCount = 0;

    @Override
    public void teleopInit() {
        try {
            SmartDashboard.putString("teleopInit", "teleopInit  count: " + m_teleopCount++);
            mLog.info("teleopInit:  ---------------------------------");
            super.teleopInit();
            Scheduler.getInstance().removeAll();
            mArcadeDriveCmd = new ArcadeDriveCmd(mControlBoard.mDriveStick);
            Scheduler.getInstance().add(mArcadeDriveCmd);
            // NavXSys.getInstance().zeroYawHeading();
            // Scheduler.getInstance().removeAll();
            // mArcadeDriveCmd = new ArcadeDriveCmd(mControlBoard.mDriveStick);
            // Scheduler.getInstance().add(mArcadeDriveCmd);
            // mDistSens = DistanceSensor.getInstance();
            // mDistSens.enable();
            // mHallSwitch = new DigitalInput(0);
            // mHallCtr = new Counter(mHallSwitch);
            // mHallCtr.reset();
            // mElvSlowCmd = new ElvMoveUpSlow();
            // Scheduler.getInstance().add(mElvSlowCmd); //mArcadeDriveCmd);
        } catch (Exception ex) {
            mLog.severe(ex, "Robot.teleopInit:  exception: " + ex.getMessage());
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        try {
            // must call the scheduler to run
            Scheduler.getInstance().run();
            // mLogPeriodic.debug("telPer: Hall Switch: %b Counter: %d period: %.3f ",
            // mHallSwitch.get(), mHallCtr.get(), mHallCtr.getPeriod());
            // mLogPeriodic.debug(mDriveSys.logMotor()); //mDriveSys.logSensors());
            // mLogPeriodic.debug(mElvSys.printPosn("telPer:"));
            // mLogPeriodic.debug(mDriveSys.logMotor()); //mDriveSys.logSensors());

            if (mWristSys != null) {
                mLogPeriodic.debug(mWristSys.printPosn("telPer:"));
            }
        } catch (Exception ex) {
            mLog.severe(ex, "Robot.teleopPeriodic:  exception: " + ex.getMessage());
        }
    }

    // ******************** test
    // ********************************************************

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
