package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import team6072.robo2019.pid.IPIDOutput;

/**
 * Class used to pass to PIDController to receive the controller output
 */
public class PIDOutTalonWithoutRamping implements PIDOutput {

    private double mPIDOutput = 0.0;
    private WPI_TalonSRX m_talon;

    public PIDOutTalonWithoutRamping(WPI_TalonSRX talon) {
        m_talon = talon;
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