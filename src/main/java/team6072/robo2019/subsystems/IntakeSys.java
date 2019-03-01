package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;



/**
 * Intake sys has a single talon, and just has to drive the wheels in or out
 * do need a sensor to detect when ball loaded and stop driving wheels in, so it does not shred the ball
 */
public class IntakeSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(IntakeSys.class.getName());

    private static IntakeSys mInstance;


    private WPI_TalonSRX mTalon;


    public static IntakeSys getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSys();
        }
        return mInstance;
    }



    public IntakeSys() {
        mLog.info("IntakeSys ctor  ----------------------------------------------");
        try {
            mTalon = new WPI_TalonSRX(RobotConfig.INTAKE_MASTER);
            mTalon.configFactoryDefault();
            mTalon.setSafetyEnabled(false);
            mTalon.setName(String.format("%d: Intake", RobotConfig.INTAKE_MASTER));
            mTalon.set(ControlMode.PercentOutput, 0);

            mLog.info("IntakeSys ctor  complete -------------------------------------");
        } catch (Exception ex) {
            mLog.severe(ex, "IntakeSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }


    @Override
    public void initDefaultCommand() {
    }


    public void intakeWheelsIn() {
        mLog.debug("intakeWheelsIn   ------------------------------------------------------");
        mTalon.set(ControlMode.PercentOutput, -0.5);
    }


    public void intakeWheelsOut() {
        mLog.debug("intakeWheelsOut   ------------------------------------------------------");
        mTalon.set(ControlMode.PercentOutput, 0.5);
    }


    public void intakeWheelsStop() {
        mLog.debug("intakeWheelsStop   ------------------------------------------------------");
        mTalon.set(ControlMode.PercentOutput, 0.0);
    }
  


}
