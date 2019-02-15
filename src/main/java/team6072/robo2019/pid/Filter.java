

package team6072.robo2019.pid;


/**
 * Superclass for filters.
 */
public abstract class Filter implements IPIDSource {


    private final IPIDSource m_source;

    public Filter(IPIDSource source) {
        m_source = source;
    }


    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        m_source.setPIDSourceType(pidSource);
    }


    @Override
    public PIDSourceType getPIDSourceType() {
        return m_source.getPIDSourceType();
    }


    @Override
    public abstract double pidGet();
    

    /**
     * Returns the current filter estimate without also inserting new data as
     * pidGet() would do.
     *
     * @return The current filter estimate
     */
    public abstract double get();
    

    /**
     * Reset the filter state.
     */
    public abstract void reset();


    /**
     * Calls PIDGet() of source.
     *
     * @return Current value of source
     */
    protected double pidGetSource() {
        return m_source.pidGet();
    }


}