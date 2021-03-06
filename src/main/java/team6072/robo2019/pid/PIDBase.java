/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.pid;

import java.util.concurrent.locks.ReentrantLock;
import static java.util.Objects.requireNonNull;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import team6072.robo2019.logging.*;




/**
 * Class implements a PID Control Loop.
 *
 * <p>
 * Creates a separate thread which reads the given IPIDSource and takes care of
 * the integral calculations, as well as writing the given IPIDOutput.
 *
 * <p>
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
@SuppressWarnings("PMD.TooManyFields")
public class PIDBase extends SendableBase implements IPID, IPIDOutput {

    protected static final LogWrapper mLog = new LogWrapper(PIDBase.class.getName());
    protected static PeriodicLogger mLogPeriodic = new PeriodicLogger(mLog, 5);

    public static final double kDefaultPeriod = 0.05;

    private static int m_instanceCount;

    // Factor for "proportional" control
    @SuppressWarnings("MemberName")
    private double m_P;

    // Factor for "integral" control
    @SuppressWarnings("MemberName")
    private double m_I;

    // Factor for "derivative" control
    @SuppressWarnings("MemberName")
    private double m_D;

    // Factor for "feed forward" control
    @SuppressWarnings("MemberName")
    private double m_F;

    // |maximum output|
    private double m_maximumOutput = 1.0;

    // |minimum output|
    private double m_minimumOutput = -1.0;

    // Maximum input - limit setpoint to this
    private double m_maximumInput;

    // Minimum input - limit setpoint to this
    private double m_minimumInput;

    // Input range - difference between maximum and minimum
    private double m_inputRange;

    // Do the endpoints wrap around? (e.g., absolute encoder)
    private boolean m_continuous;

    // Is the PID controller enabled
    protected boolean m_enabled = false;

    // The prior error (used to compute velocity)
    private double m_prevError;

    // The sum of the errors for use in the integral calc
    private double m_totalError;

    // The tolerance object used to check if on target
    private ITolerance m_tolerance;

    private double m_setpoint;
    private double m_prevSetpoint;
    @SuppressWarnings("PMD.UnusedPrivateField")
    private double m_error;
    private double m_result;

    private IPIDSource m_origSource;
    private LinearDigitalFilter m_filter;

    protected ReentrantLock m_thisMutex = new ReentrantLock();

    // Ensures when disable() is called, pidWrite() won't run if calculate()
    // is already running at that time.
    protected ReentrantLock m_pidWriteMutex = new ReentrantLock();

    protected IPIDSource m_pidInput;
    protected IPIDOutput m_pidOutput;
    protected WPITimer m_setpointTimer;
    protected IPIDExecOnTarget m_execOnTarget;



    // ------------------------ Implement Tolerance  -----------------------------------------

    /**
     * Tolerance is the type of tolerance used to specify if the PID controller is on target.
     *
     * <p>
     * The various implementations of this class such as PercentageTolerance and
     * AbsoluteTolerance specify types of tolerance specifications to use.
     */
    public interface ITolerance {
        boolean onTarget();
    }


    /**
     * Used internally for when Tolerance hasn't been set.
     */
    public static class NullTolerance implements ITolerance {
        @Override
        public boolean onTarget() {
            throw new IllegalStateException("No tolerance value set when calling onTarget().");
        }
    }


    public class PercentageTolerance implements ITolerance {
        private final double m_percentage;
        public boolean onTargLogged = false;

        PercentageTolerance(double value) {
            m_percentage = value;
        }

        @Override
        public boolean onTarget() {

            boolean onTarg = Math.abs(getError()) < m_percentage / 100 * m_inputRange;

            boolean disable = false;
            if (onTarg) {
                if (m_execOnTarget != null) {
                    disable = m_execOnTarget.PID_ExecOnTarget();
                    if (disable) {
                        m_enabled = false;
                    }
                }
                if (m_debugEnabled && !onTargLogged) {
                    onTargLogged = true;
                    double targ = getSetpoint();
                    double cur = m_pidInput.pidGet();
                    mLog.debug("%s: on Target  cur: %.3f   targ: %.3f  disabled: %b  ----------------", m_name, cur, targ, disable);
                }
            }
            return onTarg;
        }
    }


    public class AbsoluteTolerance implements ITolerance {
        private final double m_value;
        public boolean onTargLogged = false;

        AbsoluteTolerance(double value) {
            m_value = value;
        }

        @Override
        public boolean onTarget() {
            boolean onTarg = Math.abs(getError()) < m_value;
            boolean disable = false;
            if (onTarg) {
                if (m_execOnTarget != null) {
                    disable = m_execOnTarget.PID_ExecOnTarget();
                    if (disable) {
                        m_enabled = false;
                    }
                }
                if (m_debugEnabled && !onTargLogged) {
                    onTargLogged = true;
                    double targ = getSetpoint();
                    double cur = m_pidInput.pidGet();
                    mLog.debug("%s: on Target  cur: %.3f   targ: %.3f  disabled: %b  ----------------", m_name, cur,
                            targ, disable);
                }
            }
            return onTarg;
        }
    }

    // -------------------------  end Tolerance  ----------------------------------------


    protected String m_name;


    // set in setRamping()
    private boolean m_rampEnabled;          // aer we ramping
    private double m_rampStartUnits;        // how many units in start to ramp
    private double m_rampEndUnits;          // how many units in end to ramp

    // set in calculate() first time through
    private double m_startPoint;            // need to know sensor posn at start for ramping
    private boolean m_firstCalc;
    private boolean m_dirnPositive = true;  // if m_setPoint >= m_startPoint
    private double m_rampEndofStart;
    private double m_rampStartofEnd;

    // se in setBasePower()
    private double m_basePowerUp = 0.0;     // set the base power required to move system
    private double m_basePowerDown = 0.0;

    private boolean m_debugEnabled = false;

    private IPIDExecOnTarget mExecOnTarget;

    /**
     * Allocate a PID object with the given constants for P, I, D, and F.
     *
     * @param Kp     the proportional coefficient
     * @param Ki     the integral coefficient
     * @param Kd     the derivative coefficient
     * @param Kf     the feed forward term
     * @param source The IPIDSource object that is used to get values
     * @param output The IPIDOutput object that is set to the output percentage
     */
    @SuppressWarnings("ParameterName")
    public PIDBase(String name, double Kp, double Ki, double Kd, double Kf, IPIDSource source, IPIDOutput output, IPIDExecOnTarget execOnTarg) {
        super(false);
        m_name = name;
        m_execOnTarget = execOnTarg;
        requireNonNull(source, "Null IPIDSource was given");
        requireNonNull(output, "Null IPIDOutput was given");

        m_setpointTimer = new WPITimer();
        m_setpointTimer.start();

        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
        m_F = Kf;

        // Save original source
        m_origSource = source;

        // Create LinearDigitalFilter with original source as its source argument
        // having only one period effectively creates no operation
        m_filter = LinearDigitalFilter.movingAverage(m_origSource, 1);
        m_pidInput = m_filter;

        m_pidOutput = output;

        m_instanceCount++;
        HAL.report(tResourceType.kResourceType_PIDController, m_instanceCount);
        m_tolerance = new NullTolerance();
        m_rampEnabled = false;
        m_firstCalc = true;
        setName("PIDController", m_instanceCount);

        mLog.info("%s.ctor:  p: %.6f  i: %3f  d: %3f  f: %3f", name, m_P, m_I, m_D, m_F);
    }


    /**
     * Allocate a PID object with the given constants for P, I, and D.
     *
     * @param Kp     the proportional coefficient
     * @param Ki     the integral coefficient
     * @param Kd     the derivative coefficient
     * @param source the IPIDSource object that is used to get values
     * @param output the IPIDOutput object that is set to the output percentage
     */
    @SuppressWarnings("ParameterName")
    public PIDBase(String name, double Kp, double Ki, double Kd, IPIDSource source, IPIDOutput output) {
        this(name, Kp, Ki, Kd, 0.0, source, output, null);
    }


    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask and is created during initialization.
     */
    @SuppressWarnings({ "LocalVariableName", "PMD.ExcessiveMethodLength", "PMD.NPathComplexity" })
    protected void calculate() {
        if (m_origSource == null || m_pidOutput == null) {
            return;
        }

        boolean enabled;

        m_thisMutex.lock();
        try {
            enabled = m_enabled;
        } finally {
            m_thisMutex.unlock();
        }
        if (!enabled) {
            //mLog.debug("%s.calc disabled", m_name);
            return;
        }

        double input;

        // Storage for function inputs
        IPIDSource.PIDSourceType pidSourceType;
        double P;
        double I;
        double D;
        double feedForward = calculateFeedForward();
        double minimumOutput;
        double maximumOutput;
        double rampScaleFactor;

        // Storage for function input-outputs
        double prevError;
        double error;
        double totalError;

        m_thisMutex.lock();
        try {
            input = m_pidInput.pidGet();
            rampScaleFactor = 1.0; // default no ramping
            if (m_rampEnabled) {
                // if we are ramping and this is the first time calcing, need to set up
                m_dirnPositive = (m_setpoint >= input);
                if (m_firstCalc) {
                    m_firstCalc = false;
                    m_startPoint = input;
                    m_rampEndofStart = input;
                    m_rampStartofEnd = m_setpoint;
                    if (m_dirnPositive) {
                            m_rampEndofStart = input + m_rampStartUnits;
                            m_rampStartofEnd = m_setpoint - m_rampEndUnits;
                    } else {
                        m_rampEndofStart = input - m_rampStartUnits;
                        m_rampStartofEnd = m_setpoint + m_rampEndUnits;
                    }
                    mLog.debug("%s.calc.first: start: %.3f  EoS: %.3f  SoE: %.3f  set: %.3f  -------------------------",
                                m_name, m_startPoint, m_rampEndofStart, m_rampStartofEnd, m_setpoint);
                }
                if (m_dirnPositive) {                       // going up
                    if (input < m_rampEndofStart) { // inside start ramp
                        rampScaleFactor = Math.max(m_basePowerUp, Math.abs(input - m_startPoint) / m_rampStartUnits);
                    }
                    if (input > m_rampStartofEnd) {  // inside end ramp
                        rampScaleFactor = Math.abs(m_setpoint - input) / m_rampEndUnits;
                    }
                }
                else {                                      // going down
                    if (input > m_rampEndofStart) {
                        rampScaleFactor = Math.max(m_basePowerDown, Math.abs(input - m_startPoint) / m_rampStartUnits);
                    }
                    if (input < m_rampStartofEnd) {
                        rampScaleFactor = Math.abs(m_setpoint - input) / m_rampEndUnits;
                    }
                }
            }

            pidSourceType = m_pidInput.getPIDSourceType();
            P = m_P;
            I = m_I;
            D = m_D;
            minimumOutput = m_minimumOutput;
            maximumOutput = m_maximumOutput;

            prevError = m_prevError;
            error = getContinuousError(m_setpoint - input);
            totalError = m_totalError;
        } finally {
            m_thisMutex.unlock();
        }

        // Storage for function outputs
        double result;

        if (pidSourceType.equals(IPIDSource.PIDSourceType.kRate)) {
            if (P != 0) {
                totalError = clamp(totalError + error, minimumOutput / P, maximumOutput / P);
            }
            result = P * totalError + D * error + feedForward;
        } else {
            if (I != 0) {
                totalError = clamp(totalError + error, minimumOutput / I, maximumOutput / I);
            }
            result = P * error + I * totalError + D * (error - prevError) + feedForward;
        }
        if (m_debugEnabled) {
            mLogPeriodic.debug(
                    "%s.exec.calc: set: %.3f  inp: %.3f  err: %.3f  totErr: %.3f  ff: %.3f   res: %.3f   scale: %.3f",
                    m_name, m_setpoint, input, error, totalError, feedForward, result, rampScaleFactor);
        }
        result = result * rampScaleFactor;
        result = clamp(result, minimumOutput, maximumOutput);

        // Ensures m_enabled check and pidWrite() call occur atomically
        m_pidWriteMutex.lock();
        try {
            m_thisMutex.lock();
            try {
                if (m_enabled) {
                    // Don't block other PIDController operations on pidWrite()
                    m_thisMutex.unlock();
                    m_pidOutput.pidWrite(result);
                }
            } finally {
                if (m_thisMutex.isHeldByCurrentThread()) {
                    m_thisMutex.unlock();
                }
            }
        } finally {
            m_pidWriteMutex.unlock();
        }

        m_thisMutex.lock();
        try {
            m_prevError = error;
            m_error = error;
            m_totalError = totalError;
            m_result = result;
        } finally {
            m_thisMutex.unlock();
        }
    }

    /**
     * Calculate the feed forward term.
     *
     * <p>
     * Both of the provided feed forward calculations are velocity feed forwards. If
     * a different feed forward calculation is desired, the user can override this
     * function and provide his or her own. This function does no synchronization
     * because the PIDController class only calls it in synchronized code, so be
     * careful if calling it oneself.
     *
     * <p>
     * If a velocity PID controller is being used, the F term should be set to 1
     * over the maximum setpoint for the output. If a position PID controller is
     * being used, the F term should be set to 1 over the maximum speed for the
     * output measured in setpoint units per this controller's update period (see
     * the default period in this class's constructor).
     * 
     * If SourceType is kDisplacement and no change in setpoint, kF goes to zero
     */
    protected double calculateFeedForward() {
        if (m_pidInput.getPIDSourceType().equals(IPIDSource.PIDSourceType.kRate)) {
            return m_F * getSetpoint();
        } else {
            double temp = m_F * getDeltaSetpoint();
            m_prevSetpoint = m_setpoint;
            m_setpointTimer.reset();
            return temp;
        }
    }


    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return the current setpoint
     */
    @Override
    public double getSetpoint() {
        m_thisMutex.lock();
        try {
            return m_setpoint;
        } finally {
            m_thisMutex.unlock();
        }
    }

    /**
     * Set the setpoint for the PIDController.
     * If minimumInput and maximumInput have been set, they bound the setpoint
     * @param setpoint the desired setpoint
     */
    @Override
    public void setSetpoint(double setpoint) {
        m_thisMutex.lock();
        try {
            if (m_maximumInput > m_minimumInput) {
                if (setpoint > m_maximumInput) {
                    m_setpoint = m_maximumInput;
                } else if (setpoint < m_minimumInput) {
                    m_setpoint = m_minimumInput;
                } else {
                    m_setpoint = setpoint;
                }
            } else {
                m_setpoint = setpoint;
            }
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Returns the change in setpoint over time of the PIDController. m_prevSetPoint
     * is updated in caclulateFeedForward
     * 
     * @return the change in setpoint over time
     */
    public double getDeltaSetpoint() {
        m_thisMutex.lock();
        try {
            return (m_setpoint - m_prevSetpoint) / m_setpointTimer.get();
        } finally {
            m_thisMutex.unlock();
        }
    }



    /**
     * If true enable debug output from the calculate() method
     * @param enabled
     * @param period - interval between log calls before producing output
     */
    public void setDebugEnabled(boolean enabled, int period) {
        m_debugEnabled = enabled;
        if (enabled) {
            mLogPeriodic.setPeriod(period);
        }
    }


    /**
     * Enable ramping
     * @param startRampUnitLength - number of sensor units from start to ramp up for
     * @param endRampUnitLength - number of sensor units before end to start ramping down
     */
    public void setRamp(double startRampUnitLength, double endRampUnitLength) {
        m_thisMutex.lock();
        try {
            m_rampStartUnits = startRampUnitLength;
            m_rampEndUnits = endRampUnitLength;
            m_rampEnabled = true;
            mLog.debug("setRamp: start: %.3f   end: %.3f", startRampUnitLength, endRampUnitLength);
        } finally {
            m_thisMutex.unlock();
        }
    }



    /**
     * set teh base poer required to move the system up/down forward/backward
     */
    public void setBasePower(double powerUp, double powerDown) {
         m_thisMutex.lock();
        try {
            m_basePowerUp = powerUp;
            m_basePowerDown = powerDown;
            mLog.debug("setBasePower: up: %.3f   down: %.3f", powerUp, powerDown);
        } finally {
            m_thisMutex.unlock();
        } 
    }


    /**
     * Set the PID Controller gain parameters. Set the proportional, integral, and
     * differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    @Override
    @SuppressWarnings("ParameterName")
    public void setPID(double p, double i, double d) {
        m_thisMutex.lock();
        try {
            m_P = p;
            m_I = i;
            m_D = d;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the PID Controller gain parameters. Set the proportional, integral, and
     * differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     * @param f Feed forward coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setPID(double p, double i, double d, double f) {
        m_thisMutex.lock();
        try {
            m_P = p;
            m_I = i;
            m_D = d;
            m_F = f;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the Proportional coefficient of the PID controller gain.
     *
     * @param p Proportional coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setP(double p) {
        m_thisMutex.lock();
        try {
            m_P = p;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the Integral coefficient of the PID controller gain.
     *
     * @param i Integral coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setI(double i) {
        m_thisMutex.lock();
        try {
            m_I = i;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the Differential coefficient of the PID controller gain.
     *
     * @param d differential coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setD(double d) {
        m_thisMutex.lock();
        try {
            m_D = d;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the Feed forward coefficient of the PID controller gain.
     * 
     * <p>
     * If a velocity PID controller is being used, the F term should be set to 1
     * over the maximum setpoint for the output. If a position PID controller is
     * being used, the F term should be set to 1 over the maximum speed for the
     * output measured in setpoint units per this controller's update period (see
     * the default period in this class's constructor).
     *
     * @param f feed forward coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setF(double f) {
        m_thisMutex.lock();
        try {
            m_F = f;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    @Override
    public double getP() {
        m_thisMutex.lock();
        try {
            return m_P;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    @Override
    public double getI() {
        m_thisMutex.lock();
        try {
            return m_I;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    @Override
    public double getD() {
        m_thisMutex.lock();
        try {
            return m_D;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Get the Feed forward coefficient.
     *
     * @return feed forward coefficient
     */
    public double getF() {
        m_thisMutex.lock();
        try {
            return m_F;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Return the current PID result This is always centered on zero and constrained
     * the the max and min outs.
     *
     * @return the latest calculated output
     */
    public double get() {
        m_thisMutex.lock();
        try {
            return m_result;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the PID controller to consider the input to be continuous, Rather then
     * using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public void setContinuous(boolean continuous) {
        if (continuous && m_inputRange <= 0) {
            throw new IllegalStateException("No input range set when calling setContinuous().");
        }
        m_thisMutex.lock();
        try {
            m_continuous = continuous;
        } finally {
            m_thisMutex.unlock();
        }
    }

    /**
     * Set the PID controller to consider the input to be continuous, Rather then
     * using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     */
    public void setContinuous() {
        setContinuous(true);
    }


    /**
     * Sets the maximum and minimum values expected from the input and setpoint.
     *
     * @param minimumInput the minimum value expected from the input
     * @param maximumInput the maximum value expected from the input
     */
    public void setInputRange(double minimumInput, double maximumInput) {
        m_thisMutex.lock();
        try {
            if (minimumInput > maximumInput) {
                throw new BoundaryException("Lower bound is greater than upper bound");
            }
            m_minimumInput = minimumInput;
            m_maximumInput = maximumInput;
            m_inputRange = maximumInput - minimumInput;
        } finally {
            m_thisMutex.unlock();
        }

        setSetpoint(m_setpoint);
    }


    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum percentage to write to the output - default -1.0
     * @param maximumOutput the maximum percentage to write to the output - default  1.0
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        m_thisMutex.lock();
        try {
            if (minimumOutput > maximumOutput) {
                throw new BoundaryException("Lower bound is greater than upper bound");
            }
            m_minimumOutput = minimumOutput;
            m_maximumOutput = maximumOutput;
        } finally {
            m_thisMutex.unlock();
        }
    }



    /**
     * Returns the current difference of the input from the setpoint.
     *
     * @return the current error
     */
    @Override
    public double getError() {
        m_thisMutex.lock();
        try {
            return getContinuousError(getSetpoint() - m_pidInput.pidGet());
        } finally {
            m_thisMutex.unlock();
        }
    }

    /**
     * Returns the current difference of the error over the past few iterations. You
     * can specify the number of iterations to average with setToleranceBuffer()
     * (defaults to 1). getAvgError() is used for the onTarget() function.
     *
     * @deprecated Use getError(), which is now already filtered.
     * @return the current average of the error
     */
    @Deprecated
    public double getAvgError() {
        m_thisMutex.lock();
        try {
            return getError();
        } finally {
            m_thisMutex.unlock();
        }
    }

    /**
     * Sets what type of input the PID controller will use.
     *
     * @param srcType the type of input
     */
    void setIPIDSourceType(IPIDSource.PIDSourceType srcType) {
        m_pidInput.setPIDSourceType(srcType);
    }

    /**
     * Returns the type of input the PID controller is using.
     *
     * @return the PID controller input type
     */
    IPIDSource.PIDSourceType getPIDSourceType() {
        return m_pidInput.getPIDSourceType();
    }

    /**
     * Set the PID tolerance using a Tolerance object. Tolerance can be specified as
     * a percentage of the range or as an absolute value. The Tolerance object
     * encapsulates those options in an object. Use it by creating the type of
     * tolerance that you want to use: setTolerance(new
     * PIDController.AbsoluteTolerance(0.1))
     *
     * @deprecated Use setPercentTolerance() instead.
     * @param tolerance A tolerance object of the right type, e.g. PercentTolerance
     *                  or AbsoluteTolerance
     */
    @Deprecated
    public void setTolerance(ITolerance tolerance) {
        m_tolerance = tolerance;
    }

    /**
     * Set the absolute error which is considered tolerable for use with OnTarget.
     *
     * @param absvalue absolute error which is tolerable in the units of the input
     *                 object
     */
    public void setAbsoluteTolerance(double absvalue) {
        m_thisMutex.lock();
        try {
            m_tolerance = new AbsoluteTolerance(absvalue);
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the percentage error which is considered tolerable for use with OnTarget.
     * (Input of 15.0 = 15 percent)
     *
     * @param percentage percent error which is tolerable
     */
    public void setPercentTolerance(double percentage) {
        m_thisMutex.lock();
        try {
            m_tolerance = new PercentageTolerance(percentage);
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Set the number of previous error samples to average for tolerancing. When
     * determining whether a mechanism is on target, the user may want to use a
     * rolling average of previous measurements instead of a precise position or
     * velocity. This is useful for noisy sensors which return a few erroneous
     * measurements when the mechanism is on target. However, the mechanism will not
     * register as on target for at least the specified bufLength cycles.
     *
     * @deprecated Use a LinearDigitalFilter as the input.
     * @param bufLength Number of previous cycles to average.
     */
    @Deprecated
    public void setToleranceBuffer(int bufLength) {
        m_thisMutex.lock();
        try {
            m_filter = LinearDigitalFilter.movingAverage(m_origSource, bufLength);
            m_pidInput = m_filter;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInput.
     *
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget() {
        m_thisMutex.lock();
        try {
            return m_tolerance.onTarget();
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    @Override
    public void reset() {
        m_thisMutex.lock();
        try {
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
            m_firstCalc = true;
            m_rampEnabled = false;
        } finally {
            m_thisMutex.unlock();
        }
    }


    /**
     * Passes the output directly to setSetpoint().
     *
     * <p>
     * PIDControllers can be nested by passing a PIDController as another
     * PIDController's output. In that case, the output of the parent controller
     * becomes the input (i.e., the reference) of the child.
     *
     * <p>
     * It is the caller's responsibility to put the data into a valid form for
     * setSetpoint().
     */
    @Override
    public void pidWrite(double output) {
        setSetpoint(output);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.setSafeState(this::reset);
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }


    /**
     * Wraps error around for continuous inputs. The original error is returned if
     * continuous mode is disabled. This is an unsynchronized function.
     *
     * @param error The current error of the PID controller.
     * @return Error for continuous inputs.
     */
    protected double getContinuousError(double error) {
        if (m_continuous && m_inputRange > 0) {
            error %= m_inputRange;
            if (Math.abs(error) > m_inputRange / 2) {
                if (error > 0) {
                    return error - m_inputRange;
                } else {
                    return error + m_inputRange;
                }
            }
        }
        return error;
    }


    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }


}