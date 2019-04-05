package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import team6072.robo2019.pid.IPIDSource;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;


public class PIDSourceNavXYaw implements PIDSource {

    
    private NavXSys mNavXSys;

    /**
     * pass it an instance of the navXsystem
     * @param navXSys
     */
    public PIDSourceNavXYaw(){
        mNavXSys = NavXSys.getInstance();
    }
    public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }
    public double pidGet(){
        
        return mNavXSys.getYawHeading();
    }
    public void setPIDSourceType(PIDSourceType s){
    }

}