
package team6072.robo2019.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.subsystems.IntakeSys;



public class IntakeWheelsInCmd extends Command {

    private IntakeSys mSys;

    public IntakeWheelsInCmd() {
        mSys = IntakeSys.getInstance();
        requires(mSys);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mSys.intakeWheelsIn();
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
