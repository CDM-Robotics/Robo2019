/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.commands.objectives;

import edu.wpi.first.wpilibj.command.CommandGroup;

import team6072.robo2019.commands.elevator.*;
import team6072.robo2019.commands.*;
import team6072.robo2019.subsystems.*;
import team6072.robo2019.commands.pneumatics.*;
import team6072.robo2019.commands.drive.*;



public class ObjectiveCmdGrp extends CommandGroup {


    private Objective.ElvTarget m_obj;
  
    /**
     * Create a group of commands to implement moving subsystems to an objective
     * and start the group
     * @param obj
     */
    public ObjectiveCmdGrp(Objective.ElvTarget obj) {
        m_obj = obj;
        switch (m_obj) {
        case CargoshipCargo:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.CargoshipCargo));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case CargoshipHatch:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.CargoshipHatch));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case RocketCargoLo:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.RocketCargoLo));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case RocketCargoMid:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.RocketCargoMid));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case RocketCargoHi:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.RocketCargoHi));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case RocketHatchLo:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.RocketHatchLo));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case RocketHatchMid:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.RocketHatchMid));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case RocketHatchHi:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.RocketHatchHi));
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.FlatDeployPosition));
            break;
        case HatchPickUp:
            addSequential(new ElvPIDMoveToCmd(Objective.ElvTarget.CargoshipHatch));
            // addSequential(new HatchWristExtendCmd());
            // addSequential(new WristMoveToCmd(WristSys.WristTarget.CargoshipHatch));
        }
        
    }

  

}
