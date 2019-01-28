package team6072.robo2019.commands.drive;


import team6072.robo2019.logging.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.subsystems.DriveSys;

/**
 * Define a command for driving
 */
public class ArcadeDriveCmd extends Command {

    private static final LogWrapper mLog = new LogWrapper(ArcadeDriveCmd.class.getName());

    private PeriodicLogger mPeriodicLogger;

    private Joystick mStick;

    private DriveSys mDriveSys;

    /**
     * Specify the the command requires the DriveSys subsystem
     */
    public ArcadeDriveCmd(Joystick stick) {
        requires(DriveSys.getInstance());
        mStick = stick;
        mDriveSys = DriveSys.getInstance();
        mPeriodicLogger = new PeriodicLogger(mLog, 50);
    }



    /**
     * Execute is called by the scheduler until the command returns finished or the
     * OI stops requesting - for example if the whileHeld() button command is used
     */
    protected void execute() {
        double mag = mStick.getY();
        // y comes out from stick as negative when going forward, so convert
        mag = -mag;
        double yaw = mStick.getX();
        //mPeriodicLogger.debug("ArcadeDrvCmd.exec:   mag: %.3f    yaw: %.3f", mag, yaw);
        mDriveSys.arcadeDrive(mag, yaw);
    }

    /**
     * @return Return true when command is completed
     */
    @Override
    protected boolean isFinished() {
        return false;
    }

}