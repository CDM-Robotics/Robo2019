package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



/**
 * Intake sys has a single talon, and just has to drive the wheels in or out
 * do need a sensor to detect when ball loaded and stop driving wheels in, so it does not shred the ball
 */
public class LEDSys extends Subsystem {

    private static LEDSys mInstance;
    private WPI_TalonSRX mTalon;

    public static LEDSys getInstance(){
        if(mInstance == null){
            mInstance = new LEDSys();
        }
        return mInstance;
    }

    public LEDSys(){
        mTalon = new WPI_TalonSRX(RobotConfig.LED_MASTER);
        mTalon.configFactoryDefault();
    }

    public void set(double percent){
        mTalon.set(ControlMode.PercentOutput, percent);
    }

    public void killLights(){
        mTalon.set(ControlMode.PercentOutput, 0);
    }


    @Override
    public void initDefaultCommand(){
    }

}
