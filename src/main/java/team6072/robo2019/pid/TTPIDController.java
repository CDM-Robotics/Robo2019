/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.pid;


import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;



/**
 * Class implements a PID Control Loop.
 *
 * <p>
 * Creates a separate thread which reads the given PIDSource and takes care of
 * the integral calculations, as well as writing the given PIDOutput.
 *
 * <p>
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
public class TTPIDController extends PIDBase implements Controller {
    
    Notifier m_controlLoop = new Notifier(this::calculate);

    /**
     * Allocate a PID object with the given constants for P, I, D, and F.
     *
     * @param Kp     the proportional coefficient
     * @param Ki     the integral coefficient
     * @param Kd     the derivative coefficient
     * @param Kf     the feed forward term
     * @param source The PIDSource object that is used to get values
     * @param output The PIDOutput object that is set to the output percentage
     * @param periodSecs the loop time for doing calculations in seconds. This
     *               particularly affects calculations of the integral and
     *               differential terms. The default is 0.05 (50ms).
     * @param execOnTarg method implementing IPIDExecOnTarget to be called when PID is in targ
     */
    @SuppressWarnings("ParameterName")
    public TTPIDController(String name, double Kp, double Ki, double Kd, double Kf, IPIDSource source, IPIDOutput output,
            double periodSecs, IPIDExecOnTarget execOnTarg) {
        super(name, Kp, Ki, Kd, Kf, source, output, execOnTarg);
        m_controlLoop.startPeriodic(periodSecs);
    }


    /**
     * Allocate a PID object with the given constants for P, I, D and period.
     *
     * @param Kp     the proportional coefficient
     * @param Ki     the integral coefficient
     * @param Kd     the derivative coefficient
     * @param source the PIDSource object that is used to get values
     * @param output the PIDOutput object that is set to the output percentage
     * @param periodSecs the loop time for doing calculations in seconds. This
     *               particularly affects calculations of the integral and
     *               differential terms. The default is 0.05 (50ms).
     */
    @SuppressWarnings("ParameterName")
    public TTPIDController(String name, double Kp, double Ki, double Kd, IPIDSource source, IPIDOutput output, double periodSecs) {
        this(name, Kp, Ki, Kd, 0.0, source, output, periodSecs, null);
    }


    /**
     * Allocate a PID object with the given constants for P, I, D, using a 50ms period.
     *
     * @param Kp     the proportional coefficient
     * @param Ki     the integral coefficient
     * @param Kd     the derivative coefficient
     * @param source The PIDSource object that is used to get values
     * @param output The PIDOutput object that is set to the output percentage
     */
    @SuppressWarnings("ParameterName")
    public TTPIDController(String name, double Kp, double Ki, double Kd, IPIDSource source, IPIDOutput output) {
        this(name, Kp, Ki, Kd, source, output, kDefaultPeriod);
    }


    /**
     * Allocate a PID object with the given constants for P, I, D, and F using a 50ms period.
     *
     * @param Kp     the proportional coefficient
     * @param Ki     the integral coefficient
     * @param Kd     the derivative coefficient
     * @param Kf     the feed forward term
     * @param source The PIDSource object that is used to get values
     * @param output The PIDOutput object that is set to the output percentage
     */
    @SuppressWarnings("ParameterName")
    public TTPIDController(String name, double Kp, double Ki, double Kd, double Kf, IPIDSource source, IPIDOutput output) {
        this(name, Kp, Ki, Kd, Kf, source, output, kDefaultPeriod, null);
    }


    @Override
    public void close() {
        super.close();
        m_controlLoop.close();
        m_thisMutex.lock();
        try {
            m_pidOutput = null;
            m_pidInput = null;
            m_controlLoop = null;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Begin running the PIDController.
     */
    @Override
    public void enable() {
        m_thisMutex.lock();
        try {
            m_enabled = true;
        } finally {
            m_thisMutex.unlock();
        }
        mLogPeriodic.enable();
    }


    /**
     * Set the enabled state of the PIDController.
     */
    public void setEnabled(boolean enable) {
        if (enable) {
            enable();
        } else {
            disable();
        }
    }

    
    /**
     * Return true if PIDController is enabled.
     */
    public boolean isEnabled() {
        m_thisMutex.lock();
        try {
            return m_enabled;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Stop running the PIDController, this sets the output to zero before stopping.
     */
    @Override
    public void disable() {
        // Ensures m_enabled check and pidWrite() call occur 
        mLog.debug("PID.%s disable called  ------------------", m_name);
        m_pidWriteMutex.lock();
        try {
            m_thisMutex.lock();
            try {
                m_enabled = false;
            } finally {
                m_thisMutex.unlock();
            }
            m_pidOutput.pidWrite(0);
        } finally {
            m_pidWriteMutex.unlock();
        }
        mLogPeriodic.disable();
    }



    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    @Override
    public void reset() {
        disable();
        super.reset();
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("PID.enabled", this::isEnabled, this::setEnabled);
    }


}