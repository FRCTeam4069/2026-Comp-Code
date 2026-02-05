package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public final class IntakeConstants {
    public static final int UPPER_POSITION = 65;
    public static final int LOWER_POSITION = 12;

    // Dude wtf upper case for finals
    public static final double ShooterOffset = 0.30832749;

    //new
    public static final int feedSmartCurrentLimit = 40; //TODO
    public static final int artSmartCurrentLimit = 20; //TODO

    public static final SparkMaxConfig feedConfig = new SparkMaxConfig();
    public static final SparkMaxConfig artConfig = new SparkMaxConfig();


    static {
        feedConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(feedSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
    }


    static {
        artConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(artSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);

            artConfig.softLimit
            .forwardSoftLimit(0)
            .reverseSoftLimit(IntakeConstants.UPPER_POSITION)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

    }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0.0, 0.0);
    public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0);

    //TODO tbd
    //public static Constraints constraints = new Constraints(100000.0, 100000.0);
    //public static double positionTolerance = 0.02;
    //public static double velocityTolerance = 1.00;

    
}