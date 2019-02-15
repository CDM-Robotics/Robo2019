
package team6072.robo2019.pid;


/**
 * This interface allows for PIDController to automatically read from this object.
 */
public interface IPIDSource {


    /**
     * A description for the type of output value to provide to a PIDController.
     */
    public enum PIDSourceType {
        kDisplacement, kRate
    }


    /**
     * Set which parameter of the device you are using as a process control
     * variable.
     *
     * @param pidSource An enum to select the parameter.
     */
    void setPIDSourceType(PIDSourceType pidSourceType);

    /**
     * Get which parameter of the device you are using as a process control
     * variable.
     *
     * @return the currently selected PID source parameter
     */
    PIDSourceType getPIDSourceType();

    /**
     * Get the result to use in PIDController.
     *
     * @return the result to use in PIDController
     */
    double pidGet();


}