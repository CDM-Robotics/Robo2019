package team6072.robo2019.commands.RoboLord;

import edu.wpi.first.wpilibj.command.Command;
import team6072.robo2019.commands.objectives.Objective;
import team6072.robo2019.commands.objectives.Objective.ElvTarget;
import team6072.robo2019.subsystems.RoboLord;
//import org.cdm.team6072.subsystems.CmdWatchdog;
import team6072.robo2019.subsystems.DriveSys;
import team6072.robo2019.subsystems.NavXSys;

public class RoboLordCmd extends Command {

    private DriveSys mDriveSys;
    private NavXSys mNavX;
    private Objective.TargetYaw mTargetAngle;
    private RoboLord mRoboLord;

    public RoboLordCmd() {
        mNavX = NavXSys.getInstance();
        mDriveSys = DriveSys.getInstance();
        mRoboLord = RoboLord.getInstance();
        requires(mDriveSys);
    }

    @Override
    protected void initialize() {
        mRoboLord.SetObjective(new Objective(ElvTarget.RocketCargoMid, Objective.TargetYaw.TEST));
        
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
