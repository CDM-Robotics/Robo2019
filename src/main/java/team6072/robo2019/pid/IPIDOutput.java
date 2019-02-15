package team6072.robo2019.pid;


/**
 * This interface allows PIDController to write it's results to its output.
 */
@FunctionalInterface
public interface IPIDOutput {

  /**
   * Set the output to the value calculated by PIDController.
   *
   * @param output the value calculated by PIDController
   */
    void pidWrite(double output);
  
}