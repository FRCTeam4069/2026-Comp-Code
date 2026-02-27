package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodConstants {

    public static final int hoodSmartCurrentLimit = 20; //FIXME

    public static final double PASS = 0.0; //TODO
    public static final double CLOSE_SHOOT = 0.0; //TODO
    public static final double FAR_SHOOT = 0.0; //TODO
    public static final double AWAY = 0.0;


    public static final byte LAMPREY_PORT = 0; //FIXME
    public static final double LAMPREY_OFFSET =0.0; //FIXME

    public static final double lowerLimit = 0.0;
    public static final double upperLimit = 0.0;

    // public enum hoodPositions{
    //     CLOSE_SHOOT,
    //     FAR_SHOOT,
    //     PASS
    // }

    public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

    static{
        hoodConfig
            .inverted(false)
            . idleMode(IdleMode.kBrake)
            .smartCurrentLimit(hoodSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
    }


    public record HoodCoefficients(
        double kP,
        double kI,
        double kD

        ) {}

    public static HoodCoefficients hoodCoefficients = new HoodCoefficients(
        0.0,
        0.0,
        0.0

        //FIXME tune
        );
    
}
