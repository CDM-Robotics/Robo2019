
package team6072.robo2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.RobotConfig;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import team6072.robo2019.logging.*;



public class PneumaticSys extends Subsystem {

    // define the logger for this class. This should be done for every class
    private static final LogWrapper mLog = new LogWrapper(PneumaticSys.class.getName());
    private PeriodicLogger mLogPeriodic;


    private Compressor mCompressor;

    private DoubleSolenoid mDriveTrainSolenoid;

    private DoubleSolenoid mHatchWristSolenoid; 

    private DoubleSolenoid mIntakeSolenoid;

    

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
        mCompressor.start();
        mLog.debug("PneumaticSys.ctor: exit    ---------------------");
    }


    @Override
    public void initDefaultCommand() {
    }
    
    private void setDriveLo() {

        mLog.debug("PneumaticSys.setDriveLo  <<<<<");

        mDriveTrainSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    private void setDriveHi() {

        mLog.debug("PneumaticSys.setDriveHi  >>>>>");

        mDriveTrainSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

    private void setWristExtend() {

        mLog.debug("PneumaticSys.setWristExtend  >>>>>");

        mHatchWristSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    private void setWristRetract() {

        mLog.debug("PneumaticSys.setWristRetract  <<<<<");

        mHatchWristSolenoid.set(DoubleSolenoid.Value.kReverse);

    }
    
    private void setIntakeClosed() {

        mLog.debug("PneumaticSys.setIntakeClosed  <<<<<");

        mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    private void setIntakeOpen() {

        mLog.debug("PneumaticSys.setIntakeOpen  >>>>>");

        mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);

    }
}
