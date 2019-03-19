
package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import team6072.robo2019.pid.PIDSourceNavXPitch;
import team6072.robo2019.pid.TTPIDController;


public class DistanceMeasureSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(DistanceMeasureSys.class.getName());


    private double distInInches;

    private static DistanceMeasureSys mInstance;

    public static DistanceMeasureSys getInstance() {
        if (mInstance == null) {
            mInstance = new DistanceMeasureSys();
        }
        return mInstance;
    }

    
    @Override
    public void initDefaultCommand() {
    }

    }



   








































    // hi