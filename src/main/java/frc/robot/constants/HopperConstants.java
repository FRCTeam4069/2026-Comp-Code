package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class HopperConstants {

    //TODO test current limits
    public static final int feedSmartCurrentLimit = 40; 
    public static final int pivotSmartCurrentLimit = 20; 

    public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();


    static {
        hopperConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(feedSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
    }



    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0.0, 0.0);
    public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0);

    // NOTE: not sure if these are needed quite yet
    //public static Constraints constraints = new Constraints(100000.0, 100000.0);
    //public static double positionTolerance = 0.02;
    //public static double velocityTolerance = 1.00;

    
}