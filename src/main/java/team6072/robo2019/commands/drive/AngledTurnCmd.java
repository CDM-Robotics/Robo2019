package team6072.robo2019.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
//import org.cdm.team6072.subsystems.CmdWatchdog;
import team6072.robo2019.subsystems.DriveSys;
import team6072.robo2019.subsystems.NavXSys;;

public class AngledTurnCmd extends Command {

    private DriveSys mDriveSys;
    private NavXSys mNavX;
    
    public enum TurnAngle {
        RIGHT_BALL,
        RIGHT_LOWER_HATCH,
        RIGHT_UPPER_HATCH,
        LEFT_BALL,
        LEFT_LOWER_HATCH,
        LEFT_UPPER_HATCH;
    }
    
    public AngledTurnCmd() {
        mNavX = NavXSys.getInstance();
        mDriveSys = DriveSys.getInstance();
        requires(mDriveSys);

    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
