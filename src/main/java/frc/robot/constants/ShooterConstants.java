package frc.robot.constants;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterConstants {

    public static final SparkFlexConfig shooterOneLeftConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterOneRightConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterTwoLeftConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterTwoRightConfig = new SparkFlexConfig();

    static{
        shooterOneLeftConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0);

    }

     static{
        shooterOneRightConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0);

    }

    static{
        shooterTwoLeftConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0);

    }

     static{
        shooterTwoRightConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.0);

    }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0.0, 0.0); //FIXME TUNE
    public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0); //FIXME TUNE

    
}
