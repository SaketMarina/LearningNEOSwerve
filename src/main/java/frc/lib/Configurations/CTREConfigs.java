package frc.lib.Configurations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public static CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveCanCoderConfig = new CANcoderConfiguration();
        
        OpenLoopRampsConfigs openLoopRampConfigs = new OpenLoopRampsConfigs();
        openLoopRampConfigs.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        openLoopRampConfigs.TorqueOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        openLoopRampConfigs.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        ClosedLoopRampsConfigs closedLoopRampConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampConfigs.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        closedLoopRampConfigs.TorqueClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        closedLoopRampConfigs.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        
        swerveCanCoderConfig.MagnetSensor = magnetSensorConfigs;
    }
}