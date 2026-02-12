package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class HopperConstants {

    //TODO test current limits
    public static final int hopperSmartCurrentLimit = 40; 
    public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();


    static {
        hopperConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(hopperSmartCurrentLimit)
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