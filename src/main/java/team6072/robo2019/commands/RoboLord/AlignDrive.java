package team6072.robo2019.commands.RoboLord;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.commands.objectives.Objective;
import team6072.robo2019.commands.objectives.Objective.Side;
import team6072.robo2019.subsystems.RoboLord;
//import org.cdm.team6072.subsystems.CmdWatchdog;
import team6072.robo2019.subsystems.DriveSys;
import team6072.robo2019.subsystems.NavXSys;

public class AlignDrive extends Command {

    private DriveSys mDriveSys;
    private NavXSys mNavX;
    private Objective.TargetYaw mTargetAngle;
    private RoboLord mRoboLord;

    public AlignDrive() {
        mNavX = NavXSys.getInstance();
        mDriveSys = DriveSys.getInstance();
        mRoboLord = RoboLord.getInstance();
        requires(mDriveSys);
    }

    @Override
    protected void initialize() {
        mRoboLord.SetObjective(new Objective(Side.RightMid, Objective.TargetYaw.TEST));
        
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
