package team6072.robo2019.commands.drive;

import team6072.robo2019.subsystems.DriveSys;
import edu.wpi.first.wpilibj.command.Command;


public class StopHalfSpeedCmd extends Command{

    private DriveSys mSys;

    public StopHalfSpeedCmd(){
        mSys = DriveSys.getInstance();
        requires(mSys);
    }

    public void initialize(){
    }

    public void execute(){
        mSys.stopHalfSpeed();
    }

    public boolean isFinished(){
        return true;
    }
}