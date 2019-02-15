

package team6072.robo2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;

import team6072.robo2019.logging.LogWrapper;
import team6072.robo2019.subsystems.ElevatorSys;



public class ElvHoldPIDCmd extends Command {

    private static final LogWrapper mLog = new LogWrapper(ElvHoldPIDCmd.class.getName());

    private ElevatorSys mSys;


    public ElvHoldPIDCmd() {
        mSys = ElevatorSys.getInstance();
        requires(mSys);
    }


    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mLog.debug("ElvHoldPIDCmd.init:  ----------------");
        mSys.enableHoldPosnTTPID();
    }


    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        return;
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }
    

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }


    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        mLog.debug("ElvHoldPIDCmd.interrupt:  <<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>");
        mSys.disableHoldPosnTTPID();
    }

}
