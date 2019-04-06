package team6072.robo2019.logging;



public class PeriodicLogger {

    private LogWrapper mLog;

    private int mPeriod;

    private int mCallCount;

    private boolean m_enabled;


    /**
     * Only log every Nth call to the logger
     * @param logger
     * @param callCount
     */
    public PeriodicLogger(LogWrapper logger, int period) {
        mLog = logger;
        mPeriod = period;
        mCallCount = 0;
        m_enabled = true;
    }


    /**
     * Set how many times to call log before output generated
     */
    public void setPeriod(int period) {
        mPeriod = period;
    }


    public void enable() {
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
    }


    public void info(String msg) {
        if (m_enabled && ++mCallCount >= mPeriod) {
            mLog.info(msg);
            mCallCount = 0;
        }
    }

    public void infomf(String msg, Object... params) {
        if (m_enabled && ++mCallCount >= mPeriod) {
            mLog.infomf(msg, params);
            mCallCount = 0;
        }
    }

    public void info(String msg, Object... params) {
        if (m_enabled && ++mCallCount >= mPeriod) {
            mLog.info(msg, params);
            mCallCount = 0;
        }
    }

    public void debug(String msg) {
        if (m_enabled && ++mCallCount >= mPeriod) {
            mLog.debug(msg);
            mCallCount = 0;
        }
    }

    public void debugmf(String msg, Object... params) {
        if (m_enabled && ++mCallCount >= mPeriod) {
            mLog.debugmf(msg, params);
            mCallCount = 0;
        }
    }

    public void debug(String msg, Object... params) {
        if (m_enabled && ++mCallCount >= mPeriod) {
            mLog.debug(msg, params);
            mCallCount = 0;
        }
    }

}