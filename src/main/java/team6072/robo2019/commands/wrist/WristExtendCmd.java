
package team6072.robo2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.subsystems.WristSys;



/**
 * Move the elevator up very slowly until irt has moved 500 ticks from start
 * Designed for testing the 
 */
public class WristExtendCmd extends Command {


    private WristSys mSys;

    public WristExtendCmd() {
        mSys = WristSys.getInstance();
        requires(mSys);
    }


    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mSys.initExtend();
    }


    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        mSys.execExtend();
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return mSys.moveStopped();
    }


    // Called once after isFinished returns true
    @Override
    protected void end() {
    }


    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

}
