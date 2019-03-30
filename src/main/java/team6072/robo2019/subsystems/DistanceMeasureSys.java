
package team6072.robo2019.subsystems;


import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;



public class DistanceMeasureSys extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(DistanceMeasureSys.class.getName());

    @Override
    public void initDefaultCommand() {
    }



    private static double mCurDistInches;
    private static double mAvgDistLastTwo;

    protected ScheduledExecutorService mSchedulor;

    private static DistanceMeasureSys mInstance;

    private static AnalogInput m_UltrasonicSensor = new AnalogInput(0);

    public static DistanceMeasureSys getInstance() {
        if (mInstance == null) {
            mInstance = new DistanceMeasureSys();
        }
        return mInstance;
    }

    public double getCurDistInches() {
        return mCurDistInches;
    }

    /**
     * Set up to read the distance every 100 mSec
     */
    private DistanceMeasureSys() {
        mSchedulor = Executors.newScheduledThreadPool(1);
        GetDistanceTask task = new GetDistanceTask();
        mSchedulor.scheduleWithFixedDelay(task, 100, 100, TimeUnit.MILLISECONDS);
    }


    protected static class GetDistanceTask implements Runnable {
        @Override
        public void run() {
            try {
                // read voltage from dist sensor and convert to inches
                double dist = (m_UltrasonicSensor.getValue() * 5.0 * 40.3) / 4096.0;
                if (dist < 11 || dist > 100) {
                    // out of range values
                    dist = -1;
                }
                if (dist != -1 && mCurDistInches != -1) {
                    mAvgDistLastTwo = (mCurDistInches + dist) / 2;
                }
                mCurDistInches = dist;
            } catch (Exception ex) {
                mLog.severe(ex, "RL.StartObj: ");
            }
        }

        
    }



}



   








































    // hi