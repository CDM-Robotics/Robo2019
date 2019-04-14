package team6072.robo2019.pid;

/**
 * Implement a method that is called by the PID controller when it is on target
 * Method returns TRUE if we want the controller to disable itself.
 */
@FunctionalInterface
public interface IPIDExecOnTarget {

    boolean PID_ExecOnTarget();

}
