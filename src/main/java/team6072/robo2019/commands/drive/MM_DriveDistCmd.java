
package team6072.robo2019.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.subsystems.DriveSys;



public class MM_DriveDistCmd extends Command {


    private DriveSys mSys;
    private double mDistInInches;

    private static final int STABLE_ON_TARG = 5;

    public MM_DriveDistCmd(double distInInches) {
        mSys = DriveSys.getInstance();
        mDistInInches = distInInches;

        requires(mSys);
    }


    private int mCountsOnTarg = 0;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mCountsOnTarg = 0;
        mSys.initMagicMotion(mDistInInches);
    }


    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        boolean onTarg = mSys.motionMagicOnTarget();
        if (onTarg) {
            mCountsOnTarg++;
        }
        else {
            mCountsOnTarg = 0;
        }
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return mCountsOnTarg >= STABLE_ON_TARG;
    }


    // Called once after isFinished returns true
    @Override
    protected void end() {
        mSys.arcadeDrive(0, 0);
    }


    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        mSys.arcadeDrive(0, 0); // stop the wheels from continuing to spin
        super.interrupted();
    }

}
