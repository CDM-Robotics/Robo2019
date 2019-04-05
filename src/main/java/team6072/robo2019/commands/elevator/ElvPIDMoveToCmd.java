/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;


import team6072.robo2019.subsystems.ElevatorSys;
import team6072.robo2019.commands.objectives.Objective;
import team6072.robo2019.logging.LogWrapper;


public class ElvPIDMoveToCmd extends Command {

    private static final LogWrapper mLog = new LogWrapper(ElvPIDMoveToCmd.class.getName());


    private ElevatorSys mSys;

    private Objective.ElvTarget m_target;


    public ElvPIDMoveToCmd(Objective.ElvTarget target) {
        mSys = ElevatorSys.getInstance();
        requires(mSys);
        m_target = target;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mSys.initMoveToTarget(m_target);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        mSys.execMoveToTarget();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return mSys.isMoveToTargetComplete();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        mLog.debug("ElvPIDMoveToCmd: endedza obj: %s", m_target.toString());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        mLog.debug("ElvPIDMoveToCmd: interrupted obj: %s", m_target.toString());
    }

}
