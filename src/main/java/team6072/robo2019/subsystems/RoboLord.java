
package team6072.robo2019.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.sound.midi.MidiSystem;

import edu.wpi.first.wpilibj.command.Subsystem;
import team6072.robo2019.NetTblConfig;
import team6072.robo2019.RobotConfig;
import team6072.robo2019.logging.*;
import team6072.robo2019.commands.objectives.Objective;

/**
 * RoboLord is responsible for controlling robot for drive into target
 * 
 * Drive team gets robot to within 6' and 10 deg of target, then hit an objective button.
 * Objective specifies:
 *      which side of the field we are on (left or right)
 *      what the target is:
 *          cargo ship near/mid/far
 *          rocket near/mid/far, hatch/cargo, lo/mid/high
 * 
 * Controller 
 *  1. sets an intermediate target 5' from side of ship and on centerline
 *  2. checks that we have vision and distance
 *  3. takes control of robot - signals driver
 *      also sets a watchdog thread that checks that still have vision and have not been hit
 *      if lose control, cancel objective and return control to driver
 *  4. uses continuously updated yaw PID to drive robot to intermediate target
 *  5. once within tolerance, sets yaw PID to the correct yaw for the specified target
 *      and set drive PID to the correct distance for deployment
 *      At same time, starts moving elevator and wrist to correct deployment
 * 
 */
public class RoboLord extends Subsystem {

    private static final LogWrapper mLog = new LogWrapper(RoboLord.class.getName());
    private static final PeriodicLogger mPLog = new PeriodicLogger(mLog, 5);

    @Override
    public void initDefaultCommand() {
    }



    /**
     * Specify a set of states for teh process of obtaining an objective
     */
    protected enum ObjState {
        NONE, STARTING, RUNNING_T1, RUNNING_T2, CANCELLING, STOPPED
    }
    
    protected static ObjState mCurState;

    protected static Objective mCurObjective;

    protected static boolean mVisionHasTarget;


    private RoboLord mInstance = null;
    protected ScheduledExecutorService mSchedulor;
    protected static DistanceMeasureSys mDistSys;
    protected static DriveSys mDriveSys;


    protected WatchDogTask mWatchDog;
    protected StartObjTask mStartObjTask;
    protected RunT1Task mRunT1Task;



    public RoboLord getInstance() {
        if (mInstance == null) {
            mInstance = new RoboLord();
        }
        return mInstance;
    }


    /**
     * Set up the ScheduledExecutorService that will handle all the threads
     */
    private RoboLord() {
        try {
            mLog.debug("RL.ctor");
            mDriveSys = DriveSys.getInstance();
            mDistSys = DistanceMeasureSys.getInstance();
            mSchedulor = Executors.newScheduledThreadPool(5);
            mWatchDog = new WatchDogTask("dog1");
            mSchedulor.scheduleWithFixedDelay(mWatchDog, 200, 200, TimeUnit.MILLISECONDS);
            mStartObjTask = new StartObjTask();
            mSchedulor.scheduleWithFixedDelay(mStartObjTask, 200, 200, TimeUnit.MILLISECONDS);
            mRunT1Task = new RunT1Task();
            mSchedulor.scheduleWithFixedDelay(mRunT1Task, 200, 200, TimeUnit.MILLISECONDS);
            mCurState = ObjState.NONE;
            mVisionHasTarget = false;
            // add a listener to check if vison has a target
            NetTblConfig.addUpdateListener(NetTblConfig.T_VISION, NetTblConfig.KV_HAVE_TARGET, 
                    event -> {
                        mVisionHasTarget = event.value.getBoolean();
                    });
        } catch (Exception ex) {
            mLog.severe(ex, "RL.ctor");
        }
    }



    /**
     * Check if objective is same as current, and if so ignore
     * If different and other is running, cancel it and set new objective
     * If no objective running, start one
     */
    public void SetObjective(Objective obj) {
        
        if (mCurObjective != null && mCurObjective.isEqual(obj) ) {
            mLog.debug("RL.SetObj: same objective being set");
            return;
        }
        if (mCurObjective != null && mCurState != ObjState.STOPPED) {
            cancelCurObj();
        }
        mCurObjective = obj;
        mLog.debug("RL: Setting current objective: %s", mCurObjective.toString());
    }


    /**
     * Cancel the current objective
     */
    protected void cancelCurObj() {
        mLog.debug("RL: Canceling current objective: %s", mCurObjective.toString());
    }



    /**
     * Start the process of getting to an objective
     */
    protected static class StartObjTask implements Runnable {
        @Override
        public void run() {
            try {
                if (mCurState != ObjState.STARTING) {
                    return;
                }
                if (!mVisionHasTarget) {
                    // vision does not have a target
                    return;
                }
                if (mDistSys.getCurDistInches() == -1) {
                    // distance sensor not reading anything
                    return;
                }
                // looks like we have vision and distance - notify driver, slow robot, set up yaw PID
                NetTblConfig.setVal(NetTblConfig.T_VISION, NetTblConfig.KV_ROBOCONTROL, true);
                // need to calc yaw to get to T1 from where we are
                mDriveSys.setRoboControl(true);
                mDriveSys.initTurnDrive(mCurObjective.getTargetYaw(), 0.2);
                mCurState = ObjState.RUNNING_T1;
            } catch (Exception ex) {
                mLog.severe(ex, "RL.StartObj: ");
            }
        }
    }


    /**
     * Looking to move to target T1
     */
    protected static class RunT1Task implements Runnable {
        @Override
        public void run() {
            try {
                if (mCurState != ObjState.RUNNING_T1) {
                    return;
                }


            } catch (Exception ex) {
                mLog.severe(ex, "RL.RunT1Task: ");
            }
        }
    }


    /**
     * WatchDog has to check for collisions and loss of vision, and force a cancellation
     * of the objective if we loose control
     */
    protected static class WatchDogTask implements Runnable {
        private String mName;

        public WatchDogTask(String name) {
            mName = name;
            mLog.debug("RL: WatchDogTask.ctor:  %s  ----------------", mName);
        }

        @Override
        public void run() {
            try {
                if (mCurState == ObjState.NONE || mCurState == ObjState.CANCELLING || mCurState == ObjState.STOPPED) {
                    return;
                }

            } catch (Exception ex) {
                mLog.severe(ex, "RL.WatchDogTask: ");
            }
        }
    }
    

    // mRunCounter++;
    // mStatRunCounter++;
    // mLordInstanceCtr++;
    // mLordStatCtr++;
    // mPLog.debug(String.format("WatchDog.run.%s: runCtr: %d statCtr: %d
    // lordInstCtr: %d lordStatCtr: %d",
    // mName, mRunCounter, mStatRunCounter, mLordInstanceCtr, mLordStatCtr));

}
