package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class HopperConstants {

    public static final int hopperSmartCurrentLimit = 30; 
    public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();


    static {
        hopperConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(hopperSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
    }

    
}