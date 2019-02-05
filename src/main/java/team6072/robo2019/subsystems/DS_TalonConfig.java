
package team6072.robo2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Hold ALL the config for a drive Talon. CTRE doco for the class is:
 * http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_s_r_x_configuration.html
 */
public class DS_TalonConfig {

    private static TalonSRXConfiguration _config;

    public static TalonSRXConfiguration getConfig() {
        if (_config == null) {
            DS_TalonConfig instance = new DS_TalonConfig();
        }
        return _config;
    }



    private DS_TalonConfig() {

        _config = new TalonSRXConfiguration();

        /* Talon SRX */
        _config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
        _config.primaryPID.selectedFeedbackCoefficient = 0.328293;
        _config.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
        _config.auxiliaryPID.selectedFeedbackCoefficient = 0.877686;

        _config.sum0Term = FeedbackDevice.QuadEncoder;
        _config.sum1Term = FeedbackDevice.RemoteSensor0;
        _config.diff0Term = FeedbackDevice.RemoteSensor1;
        _config.diff1Term = FeedbackDevice.PulseWidthEncodedPosition;

        _config.peakCurrentLimit = 20;
        _config.peakCurrentDuration = 200;
        _config.continuousCurrentLimit = 30;
        _config.openloopRamp = 1.023000;
        _config.closedloopRamp = 1.705000;
        _config.peakOutputForward = 0.939394;
        _config.peakOutputReverse = -0.289345;
        // nominal outputs can be selected to ensure that any non-zero requested motor
        // output gets promoted to a minimum output. For example, if the nominal forward
        // is set to +0.10 (+10%), then any motor request within (0%, +10%) will be
        // promoted to +10% assuming request is beyond the neutral dead band.
        _config.nominalOutputForward = 0.739980;
        _config.nominalOutputReverse = -0.119257;
        _config.neutralDeadband = 0.199413;
        _config.voltageCompSaturation = 9.296875;
        _config.voltageMeasurementFilter = 16;
        _config.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
        _config.velocityMeasurementWindow = 8;

        // limit switches
        _config.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
        _config.reverseLimitSwitchSource = LimitSwitchSource.RemoteTalonSRX;
        _config.forwardLimitSwitchDeviceID = 6;
        _config.reverseLimitSwitchDeviceID = 5;
        _config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
        _config.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
        _config.forwardSoftLimitThreshold = 2767;
        _config.reverseSoftLimitThreshold = -1219;
        _config.forwardSoftLimitEnable = true;
        _config.reverseSoftLimitEnable = true;

        _config.slot0.kP = 504.000000;
        _config.slot0.kI = 5.600000;
        _config.slot0.kD = 0.200000;
        _config.slot0.kF = 19.300000;
        _config.slot0.integralZone = 900;
        _config.slot0.allowableClosedloopError = 217;
        _config.slot0.maxIntegralAccumulator = 254.000000;
        _config.slot0.closedLoopPeakOutput = 0.869990;
        _config.slot0.closedLoopPeriod = 33;

        _config.slot1.kP = 155.600000;
        _config.slot1.kI = 5.560000;
        _config.slot1.kD = 8.868600;
        _config.slot1.kF = 454.000000;
        _config.slot1.integralZone = 100;
        _config.slot1.allowableClosedloopError = 200;
        _config.slot1.maxIntegralAccumulator = 91.000000;
        _config.slot1.closedLoopPeakOutput = 0.199413;
        _config.slot1.closedLoopPeriod = 34;

        _config.slot2.kP = 223.232000;
        _config.slot2.kI = 34.000000;
        _config.slot2.kD = 67.000000;
        _config.slot2.kF = 6.323232;
        _config.slot2.integralZone = 44;
        _config.slot2.allowableClosedloopError = 343;
        _config.slot2.maxIntegralAccumulator = 334.000000;
        _config.slot2.closedLoopPeakOutput = 0.399804;
        _config.slot2.closedLoopPeriod = 14;

        _config.slot3.kP = 34.000000;
        _config.slot3.kI = 32.000000;
        _config.slot3.kD = 436.000000;
        _config.slot3.kF = 0.343430;
        _config.slot3.integralZone = 2323;
        _config.slot3.allowableClosedloopError = 543;
        _config.slot3.maxIntegralAccumulator = 687.000000;
        _config.slot3.closedLoopPeakOutput = 0.129032;
        _config.slot3.closedLoopPeriod = 12;

        _config.auxPIDPolarity = true;
        _config.remoteFilter0.remoteSensorDeviceID = 22;
        _config.remoteFilter0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Roll;
        _config.remoteFilter1.remoteSensorDeviceID = 41;
        _config.remoteFilter1.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
        _config.motionCruiseVelocity = 37;
        _config.motionAcceleration = 3;
        _config.motionProfileTrajectoryPeriod = 11;
        _config.feedbackNotContinuous = true;
        _config.remoteSensorClosedLoopDisableNeutralOnLOS = false;
        _config.clearPositionOnLimitF = true;
        _config.clearPositionOnLimitR = true;
        _config.clearPositionOnQuadIdx = false;
        _config.limitSwitchDisableNeutralOnLOS = true;
        _config.softLimitDisableNeutralOnLOS = false;
        _config.pulseWidthPeriod_EdgesPerRot = 9;
        _config.pulseWidthPeriod_FilterWindowSz = 32;
        _config.customParam0 = 3;
        _config.customParam1 = 5;

    }
}
