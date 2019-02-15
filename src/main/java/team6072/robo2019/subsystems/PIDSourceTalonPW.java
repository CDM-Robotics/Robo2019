package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import team6072.robo2019.pid.IPIDSource;



public class PIDSourceTalonPW implements IPIDSource {

    
    private WPI_TalonSRX mTalon;
    private IPIDSource.PIDSourceType m_sourceType;

    private int m_sensIdx;


    public PIDSourceTalonPW(WPI_TalonSRX talon, int sensIdx) {
        mTalon = talon;
        m_sourceType = IPIDSource.PIDSourceType.kDisplacement;
        m_sensIdx = sensIdx;
    }


    @Override
    public void setPIDSourceType(IPIDSource.PIDSourceType srcType) {
        m_sourceType = srcType;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return m_sourceType;
    }

    /**
     * Return the PulseWidth posn of the selected talon
     * 
     * @return
     */
    @Override
    public double pidGet() {
        return mTalon.getSelectedSensorPosition(m_sensIdx);
    }

}