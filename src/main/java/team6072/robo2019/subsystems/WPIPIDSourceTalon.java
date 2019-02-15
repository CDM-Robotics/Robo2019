/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class WPIPIDSourceTalon implements PIDSource {

    private WPI_TalonSRX mTalon;
    private PIDSourceType m_sourceType;

    public WPIPIDSourceTalon(WPI_TalonSRX talon) {
        mTalon = talon;
        m_sourceType = PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType srcType) {
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
        return mTalon.getSensorCollection().getPulseWidthPosition();
    }

}
