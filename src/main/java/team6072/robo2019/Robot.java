
package team6072.robo2019;

//import java.util.logging.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import team6072.robo2019.commands.drive.ArcadeDriveCmd;
import team6072.robo2019.subsystems.NavXSys;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;



public class Robot extends TimedRobot {

    // define the logger for this class. This should be done for every class
    private static final Logger mLog = LoggerFactory.getLogger(Robot.class);

    private ControlBoard mControlBoard;



    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        mLog.info("robotInit: Logging on ---------------------------");
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
        mLog.info("autonomousInit: -----------------------------");
    }


    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    // *********************** Teleop  *********************************************************

    @Override
    public void teleopInit() {
        mLog.info("teleopinit:  ---------------------------------");
        super.teleopInit();
        NavXSys.getInstance().zeroYawHeading();
        ArcadeDriveCmd mArcadeDriveCmd = new ArcadeDriveCmd(mControlBoard.mDriveStick);
        Scheduler.getInstance().removeAll();
        Scheduler.getInstance().add(mArcadeDriveCmd);
    }



    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    // ******************** test  ********************************************************

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
