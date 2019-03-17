/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.commands.objectives;

import edu.wpi.first.wpilibj.command.Command;



/**
 * Create a command group that will implement the objective
 */
public class ObjectiveCmd extends Command {

    Objective.ElvTarget mObjective;
    ObjectiveCmdGrp mGroup;

    public ObjectiveCmd(Objective.ElvTarget obj) {
        mObjective = obj;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        mGroup = new ObjectiveCmdGrp(mObjective);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        mGroup.start();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
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


