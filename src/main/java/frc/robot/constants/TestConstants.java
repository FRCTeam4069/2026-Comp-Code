package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class TestConstants {

    public static final SparkFlexConfig leftConfig = new SparkFlexConfig();

        static{
        leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
         }
     public static final SparkFlexConfig rightConfig = new SparkFlexConfig();

        static{
        rightConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0);
        }

        public static final double ShooterOffset = 0.30832749;
    
}
