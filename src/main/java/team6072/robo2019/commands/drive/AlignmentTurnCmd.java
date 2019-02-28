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
        mTurnAngle = mNavX.compareYawHeadings();
    }

    @Override
    protected void execute() {
        if(mTurnAngle != null)
        {
            
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

}
