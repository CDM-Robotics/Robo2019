/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.subsystems;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Class used to pass to PIDController to receive the controller output
 */
public class PIDOutReceiver implements PIDOutput {

    private double mPIDOutput = 0.0;

    public void reset() {
        mPIDOutput = 0.0;
    }

    // this method is called by the PIDController to pass the current value
    @Override
    public void pidWrite(double output) {
        mPIDOutput = output;
    }

    public double getVal() {
        return mPIDOutput;
    }

}
