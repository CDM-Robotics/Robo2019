package team6072.robo2019.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
//import org.cdm.team6072.subsystems.CmdWatchdog;
import team6072.robo2019.subsystems.DriveSys;
import team6072.robo2019.subsystems.NavXSys;;

public class AlignmentTurnCmd extends Command {

    private DriveSys mDriveSys;
    private NavXSys mNavX;
    private NavXSys.TurnAngle mTurnAngle;

    public AlignmentTurnCmd() {
        mNavX = NavXSys.getInstance();
        mDriveSys = DriveSys.getInstance();
        requires(mDriveSys);
    }

    @Override
    protected void initialize() {
        if (mTurnAngle != null) {
            mDriveSys.initTurnDrive(mNavX.compareYawHeadings().getAngle());
        }
    }

    @Override
    protected void execute() {
        if (mTurnAngle != null) {
            mDriveSys.arcadeTurnPID();
        }
    }

    @Override
    protected boolean isFinished() {
        return mDriveSys.isFinishedTurning();
    }

}
