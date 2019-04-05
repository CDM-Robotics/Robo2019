package team6072.robo2019.commands.drive;

import team6072.robo2019.subsystems.DriveSys;
import edu.wpi.first.wpilibj.command.Command;


public class StartHalfSpeedCmd extends Command{

    private DriveSys mSys;

    public StartHalfSpeedCmd(){
        mSys = DriveSys.getInstance();
        requires(mSys);
    }

    public void initialize(){
    }

    public void execute(){
        mSys.startHalfSpeed();
    }

    public boolean isFinished(){
        return true;
    }
}