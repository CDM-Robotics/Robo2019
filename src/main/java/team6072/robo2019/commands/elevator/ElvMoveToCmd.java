/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;


import team6072.robo2019.subsystems.ElevatorSys;;


public class ElvMoveToCmd extends Command {


    private ElevatorSys mSys;

    private ElevatorSys.Target m_target;


    public ElvMoveToCmd(ElevatorSys.Target target) {
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

}
