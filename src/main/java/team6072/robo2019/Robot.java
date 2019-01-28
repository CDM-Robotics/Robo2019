package team6072.robo2019;

import java.io.*;
import java.util.logging.*;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

import team6072.robo2019.logging.*;
import team6072.robo2019.commands.drive.ArcadeDriveCmd;
import team6072.robo2019.commands.drive.DriveDistCmd;
import team6072.robo2019.subsystems.DriveSys;
import team6072.robo2019.subsystems.NavXSys;



public class Robot extends TimedRobot {

    // define the logger for this class. This should be done for every class
    private static LogWrapper mLog;

    private ControlBoard mControlBoard;

    private DriveSys mDriveSys;

    private NavXSys mNavXsys;



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
            //     System.out.println(line);
            // }
            // br.close();
            FileInputStream configFile = new FileInputStream(logFile);
            LogManager.getLogManager().readConfiguration(configFile);
        } catch (IOException ex) {
            System.out.println("WARNING: Could not open configuration file");
            System.out.println("WARNING: Logging not configured (console output only)");
        }
        mLog = new LogWrapper(Robot.class.getName());
        try {
            mLog.info("robotInit: ---------------------------------------------------");
            mControlBoard = ControlBoard.getInstance();
            mDriveSys = DriveSys.getInstance();
            mNavXsys = NavXSys.getInstance();
            mDriveDistCmd = new DriveDistCmd(5);
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
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    private PeriodicLogger mDSSensors;

    @Override
    public void disabledInit() {
        mLog.info("Robot.disabledInit");
        mDSSensors = new PeriodicLogger(mLog, 50 * 5);
    }

    /**
     * gets called every 20 mSec when disabeld
     */
    @Override
    public void disabledPeriodic() {
        //mLog.info("Robot.disabledPeriodic");
        //mDSSensors.debug(mDriveSys.logSensors());
    }

    // *********************** Autonomous *********************************************************

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
            mDriveDistCmd.start();
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
        mDSSensors.debug(mDriveSys.logSensors());
    }


    // *********************** Teleop  *********************************************************

    ArcadeDriveCmd mArcadeDriveCmd;
    DriveDistCmd mDriveDistCmd;

    @Override
    public void teleopInit() {
        try {
            mLog.info("teleopInit:  ---------------------------------");
            super.teleopInit();
            NavXSys.getInstance().zeroYawHeading();
            mArcadeDriveCmd = new ArcadeDriveCmd(mControlBoard.mDriveStick);
            Scheduler.getInstance().removeAll();
            Scheduler.getInstance().add(mArcadeDriveCmd);
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
            mDSSensors.debug(mDriveSys.logSensors());
        } catch (Exception ex) {
            mLog.severe(ex, "Robot.teleopPeriodic:  exception: " + ex.getMessage());
        }
    }

    // ******************** test  ********************************************************

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
