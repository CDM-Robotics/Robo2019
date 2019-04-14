
package team6072.robo2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;

import java.awt.Robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import team6072.robo2019.logging.*;



public class PneumaticSys extends Subsystem {

    // define the logger for this class. This should be done for every class
    private static final LogWrapper mLog = new LogWrapper(PneumaticSys.class.getName());

    private PeriodicLogger mLogPeriodic;


    private Compressor mCompressor;

    private DoubleSolenoid mDriveTrainSolenoid;

    private DoubleSolenoid mHatchWristSolenoid; 

    private DoubleSolenoid mIntakeSolenoid;

    private DoubleSolenoid mIntakeLockSolenoid;

    

    private static PneumaticSys mInstance;

    public static PneumaticSys getInstance() {
        try {
            if (mInstance == null) {
                mInstance = new PneumaticSys();
            }
            return mInstance;
        } catch (Exception ex) {
            mLog.severe(ex, "PneumaticSys.ctor exception: " + ex.getMessage());
            throw ex;
        }
    }


    private PneumaticSys() {
        mLog.debug("PneumaticSys.ctor: -----------------------------");
        mCompressor = new Compressor(RobotConfig.PCM_ID);
        if (RobotConfig.IS_ROBO_2019) {
            mCompressor.start();
            mDriveTrainSolenoid = new DoubleSolenoid(RobotConfig.PCM_ID, RobotConfig.PCM_DRIVE_HIGH,
                    RobotConfig.PCM_DRIVE_LOW);
            mHatchWristSolenoid = new DoubleSolenoid(RobotConfig.PCM_ID, RobotConfig.PCM_HATCH_EXTEND,
                    RobotConfig.PCM_HATCH_RETRACT);
            mIntakeSolenoid = new DoubleSolenoid(RobotConfig.PCM_ID, RobotConfig.PCM_INTAKE_OPEN,
                    RobotConfig.PCM_INTAKE_CLOSED);
            mIntakeLockSolenoid = new DoubleSolenoid(RobotConfig.PCM_ID, RobotConfig.PCM_INTAKELOCK_OPEN,
                    RobotConfig.PCM_INTAKELOCK_CLOSED);
        }
        mLog.debug("PneumaticSys.ctor: exit    ---------------------");
    }


    @Override
    public void initDefaultCommand() {
    }
    
    public void setDriveLo() {
        mLog.debug("PneumaticSys.setDriveLo  <<<<<");
        mDriveTrainSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setDriveHi() {
        mLog.debug("PneumaticSys.setDriveHi  >>>>>");
        //mDriveTrainSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void setWristExtend() {
        mLog.debug("PneumaticSys.setWristExtend  >>>>>");
        mHatchWristSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setWristRetract() {
        mLog.debug("PneumaticSys.setWristRetract  <<<<<");
        mHatchWristSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void setFlowerOpen() {
        mLog.debug("PneumaticSys.setIntakeClosed  <<<<<");
        mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setFlowerClose() {
        mLog.debug("PneumaticSys.setIntakeOpen  >>>>>");
        mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }


    public void setIntakeLockOpen() {
        mLog.debug("PneumaticSys.setIntakeLockClosed  <<<<<");
        mIntakeLockSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setIntakeLockClose() {
        mLog.debug("PneumaticSys.setIntakLockeOpen  >>>>>");
        mIntakeLockSolenoid.set(DoubleSolenoid.Value.kReverse);
    }


}
