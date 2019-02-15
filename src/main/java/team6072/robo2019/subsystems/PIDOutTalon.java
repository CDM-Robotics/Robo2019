package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import team6072.robo2019.pid.IPIDOutput;

/**
 * Class used to pass to PIDController to receive the controller output
 */
public class PIDOutTalon implements IPIDOutput {

    private double mPIDOutput = 0.0;
    private double m_baseOutput;
    private double m_minOut;
    private double m_maxOut;
    private WPI_TalonSRX m_talon;

    public PIDOutTalon(WPI_TalonSRX talon, double baseOutput, double minOut, double maxOut) {
        m_talon = talon;
        m_baseOutput = baseOutput;
        m_minOut = minOut;
        m_maxOut = maxOut;
    }


    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }


    public void reset() {
        mPIDOutput = 0.0;
    }


    // this method is called by the PIDController to pass the current value
    @Override
    public void pidWrite(double output) {
        mPIDOutput = m_baseOutput + output;
        m_talon.set(ControlMode.PercentOutput, clamp(mPIDOutput, m_minOut, m_maxOut));
    }

    public double getVal() {
        return mPIDOutput;
    }

}