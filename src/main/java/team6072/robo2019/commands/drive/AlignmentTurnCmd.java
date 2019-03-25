package team6072.robo2019.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.commands.objectives.Objective;
//import org.cdm.team6072.subsystems.CmdWatchdog;
import team6072.robo2019.subsystems.DriveSys;
import team6072.robo2019.subsystems.NavXSys;

public class AlignmentTurnCmd extends Command {

    private DriveSys mDriveSys;
    private NavXSys mNavX;
    private Objective.TargetYaw mTargetAngle;

    public AlignmentTurnCmd(Objective.TargetYaw turn) {
        mNavX = NavXSys.getInstance();
        mDriveSys = DriveSys.getInstance();
        mTargetAngle = turn;
        requires(mDriveSys);
    }

    @Override
    protected void initialize() {
        if (mTargetAngle != null) {
            mDriveSys.initTurnDrive(mTargetAngle);
        }
    }

    @Override
    protected void execute() {
        if (mTargetAngle != null) {
            mDriveSys.arcadeTurnPID();
        }
    }

    @Override
    protected boolean isFinished() {
        return mDriveSys.isFinishedTurning();
    }

}
