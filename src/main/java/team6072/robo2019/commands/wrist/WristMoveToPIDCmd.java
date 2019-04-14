
package team6072.robo2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.subsystems.WristSys;



public class WristMoveToPIDCmd extends Command {

    private WristSys mSys;

    private WristSys.WristTarget m_target;

    public WristMoveToPIDCmd(WristSys.WristTarget target) {
        mSys = WristSys.getInstance();
        requires(mSys);
        m_target = target;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mSys.initMoveToTarget(m_target);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
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
    }

}
