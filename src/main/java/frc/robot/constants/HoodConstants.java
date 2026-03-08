package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodConstants {

    public static final int hoodSmartCurrentLimit = 40; 

    public static final double PASS = 10.0; // 17 but 10 for testing 
    public static final double CLOSE_SHOOT = 5.0; //TODO
    public static final double FAR_SHOOT = 10.0; //TODO
    public static final double AWAY = 0.0;

    public static final double lowerLimit = 0;
    public static final double upperLimit = 18;

    // public enum hoodPositions{
    //     CLOSE_SHOOT,
    //     FAR_SHOOT,
    //     PASS
    // }

    public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

    static{
        hoodConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(hoodSmartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);

        hoodConfig.softLimit
          .forwardSoftLimit(18)
          .reverseSoftLimit(0)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);

    }


    public record HoodCoefficients(
        double kP,
        double kI,
        double kD

        ) {}

    public static HoodCoefficients hoodCoefficients = new HoodCoefficients(
        1.2,
        0.0,
        0.0

        //FIXME tune
        );
    
}
