package team6072.robo2019.commands.elevator;

import team6072.robo2019.subsystems.ElevatorSys;
import edu.wpi.first.wpilibj.command.Command;

public class ElvPIDClimbCmd extends Command{

    private ElevatorSys mSys;

    public ElvPIDClimbCmd(){
    }

    @Override
    public void initialize(){
        mSys = ElevatorSys.getInstance();
        requires(mSys);
        // mSys.initNavXClimbPID();
    }
    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}