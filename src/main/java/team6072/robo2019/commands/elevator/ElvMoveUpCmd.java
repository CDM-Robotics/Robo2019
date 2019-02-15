
package team6072.robo2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.subsystems.ElevatorSys;



/**
 * Move the elevator up very slowly until irt has moved 500 ticks from start
 * Designed for testing the 
 */
public class ElvMoveUpCmd extends Command {


    private ElevatorSys mSys;

    public ElvMoveUpCmd() {
        mSys = ElevatorSys.getInstance();
        requires(mSys);
    }


    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mSys.initMoveUp();
    }


    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        mSys.execMoveUp();
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
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
