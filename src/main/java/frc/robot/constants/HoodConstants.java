package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodConstants {

    public static final int hoodSmartCurrentLimit = 20; //FIXME

     public static final double SHOOT =0.0; //TODO
     public static final double PASS = 0.0; //TODO

    public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

    static{
        hoodConfig
            .inverted(false)
            . idleMode(IdleMode.kBrake)
            .smartCurrentLimit(hoodSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
    }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0.0, 0.0); //FIXME TUNE
    
}
