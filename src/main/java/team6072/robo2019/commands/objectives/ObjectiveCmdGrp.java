/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.commands.objectives;

import edu.wpi.first.wpilibj.command.CommandGroup;

import team6072.robo2019.commands.elevator.*;
import team6072.robo2019.commands.wrist.*;
import team6072.robo2019.subsystems.*;



public class ObjectiveCmdGrp extends CommandGroup {


    private Objective m_obj;
  
    /**
     * Create a group of commands to implement moving subsystems to an objective
     * and start the group
     * @param obj
     */
    public ObjectiveCmdGrp(Objective obj) {
        m_obj = obj;
        switch (m_obj) {
        case CargoshipCargo:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.CargoshipCargo));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.CargoshipCargo));
            break;
        case CargoshipHatch:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.CargoshipHatch));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.CargoshipHatch));
            break;
        case RocketCargoLo:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.RocketCargoLo));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.RocketCargoLo));
            break;
        case RocketCargoMid:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.RocketCargoMid));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.RocketCargoMid));
            break;
        case RocketCargoHi:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.RocketCargoHi));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.RocketCargoHi));
            break;
        case RocketHatchLo:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.RocketHatchLo));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.RocketHatchLo));
            break;
        case RocketHatchMid:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.RocketHatchMid));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.RocketHatchMid));
            break;
        case RocketHatchHi:
            addSequential(new ElvMoveToCmd(ElevatorSys.ElvTarget.RocketHatchHi));
            addSequential(new WristMoveToCmd(WristSys.WristTarget.RocketHatchHi));
            break;
        }
        this.start();
    }

  

}
